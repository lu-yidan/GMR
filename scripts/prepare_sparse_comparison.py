"""Prepare matched no-IK 3/5-body targets and existing v10 baselines."""
import argparse
import hashlib
import json
import time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from duobox_to_g1_pair import NAMES, load_pair, GeneralMotionRetargeting

p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);a=p.parse_args()
a.out.mkdir(parents=True,exist_ok=False)
ws=Path(__file__).resolve().parents[2]
deploy=ws/'RoboMimic_Deploy-scalebfm'
meta=json.loads((ws/'ScaleBFM/artifacts/scalebfm/humanoid_transformer_m/model_22200_tensorrt_metadata.json').read_text())
solver=GeneralMotionRetargeting('fbx_offline','unitree_g1',actual_human_height=1.8,solver='daqp',verbose=False)
robot_names=['pelvis','left_wrist_yaw_link','right_wrist_yaw_link','left_ankle_roll_link','right_ankle_roll_link']
human_names=['Hips','LeftHand','RightHand','LeftToeBase','RightToeBase']
indices=[NAMES.index(n) for n in human_names]
offsets=np.array([solver.pos_offsets1[n] for n in human_names])
rots=Rotation.from_quat([solver.rot_offsets1[n].as_quat() for n in human_names])
scales=np.array([solver.human_scale_table[n] for n in human_names])[:,None]
# Toe is a fixed child of ankle, so invert its rigid transform; no IK.
for j,side in [(3,'left'),(4,'right')]:
 toe=solver.model.body(side+'_toe_link');ankle=solver.model.body(side+'_ankle_roll_link')
 assert int(toe.parentid[0])==ankle.id
 np.testing.assert_allclose(toe.quat,[1,0,0,0])
 offsets[j]-=toe.pos
cases=[]
for name in ['000_liu_guard','003_ma_cross','007_ma_rear_hook','021_ma_circle','027_ma_moving_combo']:
 baseline=deploy/'outputs/duobox_scalebfm/batch15_v1/motions'/f'{name}.npz'
 info=json.loads(baseline.with_suffix('.json').read_text())
 names,actors,_=load_pair(Path(info['retarget']['source']))
 pos,quat=actors[names.index(info['actor'])]
 cases.append((name,pos,quat,baseline,'recorded DuoBox',info['retarget']['source']))
for clip in [3,4]:
 job=ws/'GMR/motion_data/duobox_web/motions'/f'{clip:03d}_reactive_stream_v1'
 h=dict(np.load(job/'human_0120.npz'));pos=h['positions'][:,0]
 quat=Rotation.from_matrix(h['rotations'][:,0].reshape(-1,3,3)).as_quat(scalar_first=True).reshape(-1,21,4)
 cases.append((f'reactive_{clip:03d}',pos,quat,job/'reference_0120.npz','Ready-to-React generated',str(job/'human_0120.npz')))
manifest=[]
for name,hpos,hquat,baseline,kind,source in cases:
 base=dict(np.load(baseline));n=len(base['joint_pos']);tt=np.arange(n)/50;ts=np.arange(len(hpos))/30
 assert tt[-1]<=ts[-1]+1e-7
 positions=[];quats=[];lat=[]
 for i in range(len(hpos)):
  start=time.perf_counter();root=hpos[i,0]
  rr=Rotation.from_quat(hquat[i,indices][:,[1,2,3,0]])*rots
  pp=(hpos[i,indices]-root)*scales+root*.9+rr.apply(offsets)
  positions.append(pp);quats.append(rr.as_quat(scalar_first=True))
  if i>=8:lat.append((time.perf_counter()-start)*1000)
 positions=np.array(positions);quats=np.array(quats)
 sparse={k:v.copy() for k,v in base.items()}
 sparse['joint_pos'][:]=0;sparse['joint_vel'][:]=0
 sparse['body_pos_w'][:]=0;sparse['body_quat_w'][:]=0;sparse['body_quat_w'][:,:,0]=1
 sparse['body_lin_vel_w'][:]=0;sparse['body_ang_vel_w'][:]=0
 for j,body in enumerate(robot_names):
  bi=meta['body_names'].index(body)
  sparse['body_pos_w'][:,bi]=np.column_stack([np.interp(tt,ts,positions[:,j,k]) for k in range(3)])
  sparse['body_quat_w'][:,bi]=Slerp(ts,Rotation.from_quat(quats[:,j][:,[1,2,3,0]]))(tt).as_quat()[:,[3,0,1,2]]
 sparse['partial_reference']=np.array(True)
 sparse['valid_body_names']=np.array(robot_names)
 target=a.out/f'{name}_direct.npz';np.savez_compressed(target,**sparse)
 manifest.append(dict(name=name,kind=kind,source=str(source),baseline=str(baseline),baseline_sha256=hashlib.sha256(baseline.read_bytes()).hexdigest(),direct=str(target.resolve()),frames=n,seconds=n/50,mapping_median_ms=float(np.median(lat)),mapping_p95_ms=float(np.percentile(lat,95))))
(a.out/'manifest.json').write_text(json.dumps(manifest,indent=2))
print(json.dumps(manifest,indent=2))
