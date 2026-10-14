"""Orchestrate three existing environments through bounded causal chunk files."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import numpy as np
from scipy.spatial.transform import Rotation
from duobox_to_g1_pair import NAMES, GeneralMotionRetargeting
from duobox_continuity import add_continuity, solve_frame
from duobox_geometry import GeometryRepair


def main():
    p=argparse.ArgumentParser(); p.add_argument('--job',type=Path,required=True); a=p.parse_args()
    job=a.job.resolve(); root=Path(__file__).resolve().parents[1]; workspace=root.parent
    request=json.loads((job/'request.json').read_text())
    (job/'metadata.json').write_text(json.dumps({'method':'streaming GMR body-anchor v10'}))
    env=dict(os.environ,OMP_NUM_THREADS='1',WANDB_MODE='disabled',MUJOCO_GL='egl')
    logs=[]; workers=[]
    try:
        for name,python,script,cwd,args in [
            ('generator',workspace/'ready_to_react/.venv/bin/python',workspace/'ready_to_react/scripts/reactive_stream_worker.py',workspace/'ready_to_react',['--request',str(job/'request.json')]),
            ('tracker',Path('/home/d080/miniconda3/envs/robomimic/bin/python'),workspace/'RoboMimic_Deploy-scalebfm/tools/reactive_stream_tracker.py',workspace/'RoboMimic_Deploy-scalebfm',['--job',str(job)])]:
            log=(job/(name+'.log')).open('w'); logs.append(log)
            workers.append(subprocess.Popen([str(python),str(script),*args],cwd=cwd,env=env,stdout=log,stderr=subprocess.STDOUT))
        solvers=[]
        for _ in range(2):
            solver=GeneralMotionRetargeting('fbx_offline','unitree_g1',actual_human_height=1.8,solver='daqp',verbose=False)
            solver.set_ground_offset(0.)
            continuity,posture=add_continuity(solver)
            solvers.append((solver,continuity,posture,GeometryRepair(solver,body_anchor=True)))
        qs=[[],[]]; last=0; since=time.monotonic()
        while True:
            if (job/'abort').exists(): raise RuntimeError((job/'abort').read_text())
            for name,worker in zip(('generator','tracker'),workers):
                if worker.poll() not in (None,0): raise RuntimeError(name+' failed; see '+name+'.log')
            chunks=[x for x in sorted(job.glob('human_*.npz')) if int(x.stem.split('_')[1])>last]
            if not chunks:
                if all(w.poll()==0 for w in workers): break
                if time.monotonic()-since>600: raise TimeoutError('Pipeline stalled')
                time.sleep(.1); continue
            chunk=chunks[0]; end=int(chunk.stem.split('_')[1]); human=dict(np.load(chunk))
            begun=time.perf_counter()
            for frame in range(last,end):
                for actor,(solver,continuity,posture,geometry) in enumerate(solvers):
                    quat=Rotation.from_matrix(human['rotations'][frame,actor]).as_quat(scalar_first=True)
                    target={name:(human['positions'][frame,actor,j].copy(),quat[j].copy()) for j,name in enumerate(NAMES)}
                    q=solve_frame(solver,target,geometry)
                    if not np.isfinite(q).all(): raise ValueError('Nonfinite GMR pose')
                    qs[actor].append(q.copy()); continuity.previous=q.copy(); posture.set_target(q)
            temp=job/'robot.tmp.npz'
            np.savez_compressed(temp,qpos=np.stack(qs,1),fps=30,actors=human['actors'],
                                observed_until=human['observed_until'],prediction_ms=human['prediction_ms'],
                                retarget_ms=(time.perf_counter()-begun)*1000)
            temp.replace(job/('robot_%04d.npz'%end)); last=end; since=time.monotonic()
            print('RETARGET',end,flush=True)
        if not (job/'pair_motion.npz').exists(): raise RuntimeError('No physical trajectory produced')
        (job/'pipeline.done').touch()
    except Exception as exc:
        (job/'abort').write_text(str(exc)); raise
    finally:
        for worker in workers:
            if worker.poll() is None: worker.terminate()
        for worker in workers:
            try: worker.wait(timeout=10)
            except subprocess.TimeoutExpired: worker.kill(); worker.wait()
        for log in logs: log.close()

if __name__=='__main__': main()
