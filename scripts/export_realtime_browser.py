"""Export a finished wall-clock run; opponent retarget is visualization only."""
import argparse,json
from pathlib import Path
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation,Slerp
from duobox_to_g1_pair import GeneralMotionRetargeting,NAMES,load_pair
p=argparse.ArgumentParser();p.add_argument('--job',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
root=Path(__file__).resolve().parents[1];ws=root.parent
report=json.loads((a.job/'report.json').read_text());request=json.loads((a.job/'request.json').read_text())
h=dict(np.load(a.job/'human.npz'));q=np.load(a.job/'trace.npz')['qpos'];actors=h['actors'].tolist()
a.out.mkdir(parents=True,exist_ok=True)
if (a.out/'pair_motion.npz').exists():raise FileExistsError('Preserve existing browser result')
gmodel=mujoco.MjModel.from_xml_path(str(root/'assets/unitree_g1/g1_mocap_29dof.xml'))
dmodel=mujoco.MjModel.from_xml_path(str(ws/'RoboMimic_Deploy-scalebfm/g1_description/scene.xml'))
gids=[gmodel.joint(dmodel.joint(i).name).qposadr[0] for i in range(1,dmodel.njnt)]
blue=np.zeros((len(q),gmodel.nq));blue[:,:7]=q[:,:7];blue[:,gids]=q[:,7:]
# Reuse a whole-clip v10 cache if available; never put opponent IK on control path.
clip=request['clip'];candidates=[root/f'motion_data/duobox_web/motions/{clip:03d}_geometry_v10/pair_motion.npz',ws/f'RoboMimic_Deploy-scalebfm/outputs/duobox_scalebfm/batch15_v1/retarget/{clip:03d}_v10/pair_motion.npz']
source=next((p for p in candidates if p.exists()),None)
if source:
 ar=dict(np.load(source));orange=ar['qpos'][:,ar['actors'].tolist().index(actors[1])];orange_time=np.arange(len(orange))/30-1/30
else:
 solver=GeneralMotionRetargeting('fbx_offline','unitree_g1',actual_human_height=1.8,solver='daqp',verbose=False)
 orange=[]
 for pp,rr in zip(h['positions'][:,1],h['rotations'][:,1]):
  qq=Rotation.from_matrix(rr).as_quat(scalar_first=True);orange.append(solver.retarget({n:(pp[j],qq[j]) for j,n in enumerate(NAMES)}).copy())
 orange=np.array(orange);orange_time=np.arange(len(orange))/30
# FixedPose starts at zero yaw; the runtime aligns source pelvis to that heading.
rot=Rotation.from_matrix(h['rotations'][0,0,0])*Rotation.from_quat([-.5,-.5,-.5,.5])
m=rot.as_matrix();align=Rotation.from_euler('z',-np.arctan2(m[1,0],m[0,0]))
origin=h['positions'][0,0,0]*.9;origin[2]=0
world=report.get('world_alignment')
if world:
 align=Rotation.from_quat(world['quat_xyzw']);origin=np.array(world['source_origin'])
orange[:,:3]=align.apply(orange[:,:3]-origin)+(np.array(world['origin']) if world else 0)
orange[:,3:7]=(align*Rotation.from_quat(orange[:,[4,5,6,3]])).as_quat()[:,[3,0,1,2]]
times=np.arange(int((len(q)-1)*30/50)+1)/30
bt=np.arange(len(q))/50
b=np.column_stack([np.interp(times,bt,blue[:,j]) for j in range(gmodel.nq)])
b[:,3:7]=Slerp(bt,Rotation.from_quat(blue[:,[4,5,6,3]]))(times).as_quat()[:,[3,0,1,2]]
ot=np.clip(times+request.get('delay_ms',report['delay_ms'])/1000,orange_time[0],orange_time[-1])
o=np.column_stack([np.interp(ot,orange_time,orange[:,j]) for j in range(gmodel.nq)])
o[:,3:7]=Slerp(orange_time,Rotation.from_quat(orange[:,[4,5,6,3]]))(ot).as_quat()[:,[3,0,1,2]]
labels=[actors[0]+' · 实时3部位物理跟踪',actors[1]+' · 预录对手（仅显示）']
np.savez_compressed(a.out/'pair_motion.npz',qpos=np.stack([b,o],axis=1),fps=30,actors=labels)
meta=dict(actors=labels,fps=30,frames=len(times),seconds=len(times)/30,dynamics=True,diagnostics={},failure=report['failure'] or (None if report['completed'] else 'generation_incomplete'),wall_seconds=len(q)/50/report['actual_realtime_factor'],online_realtime=True,upper_actual_state_feedback=False,
 note=f"3 部位无 IK；实时调度运行结果回放；参考延迟 {report['delay_ms']:.0f} ms；当前目标保持 {report['current_target_hold_frames']}/{report['active_frames']} 帧；控制超时 {report['deadline_misses']} 次。橙色仅离线显示，无双方碰撞、无机器人状态回传上层。",report=str((a.job/'report.json').resolve()))
(a.out/'metadata.json').write_text(json.dumps(meta,ensure_ascii=False,indent=2));(a.out/'pipeline.done').touch()
print(a.out)
