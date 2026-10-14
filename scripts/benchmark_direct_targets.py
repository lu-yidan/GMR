"""Experimental pelvis/hands reference: GMR scale/axis offsets, no IK solve."""
import json,time,argparse
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
from duobox_to_g1_pair import NAMES,GeneralMotionRetargeting
parser=argparse.ArgumentParser();parser.add_argument('--clip',type=int,default=3);args=parser.parse_args()
root=Path(__file__).resolve().parents[1]
h=dict(np.load(root/f'motion_data/duobox_web/motions/{args.clip:03d}_reactive_stream_v1/human_0120.npz'))
meta=json.loads((root.parent/'ScaleBFM/artifacts/scalebfm/humanoid_transformer_m/model_22200_tensorrt_metadata.json').read_text())
solver=GeneralMotionRetargeting('fbx_offline','unitree_g1',actual_human_height=1.8,solver='daqp',verbose=False)
selected={'pelvis':'Hips','left_wrist_yaw_link':'LeftHand','right_wrist_yaw_link':'RightHand'}
positions=[];quats=[];samples=[]
for i in range(len(h['positions'])):
 start=time.perf_counter();q=Rotation.from_matrix(h['rotations'][i,0]).as_quat(scalar_first=True)
 target={n:(h['positions'][i,0,j].copy(),q[j].copy()) for j,n in enumerate(NAMES)}
 solver.update_targets(target)
 positions.append([solver.scaled_human_data[n][0].copy() for n in selected.values()])
 quats.append([solver.scaled_human_data[n][1].copy() for n in selected.values()])
 if i>=8:samples.append((time.perf_counter()-start)*1000)
times=np.arange(len(positions))/30;tt=np.arange(int(times[-1]*50)+1)/50
positions=np.array(positions);quats=np.array(quats);n=len(tt);b=len(meta['body_names'])
pos=np.zeros((n,b,3));rot=np.zeros((n,b,4));rot[:,:,0]=1
for j,name in enumerate(selected):
 idx=meta['body_names'].index(name)
 pos[:,idx]=np.column_stack([np.interp(tt,times,positions[:,j,k]) for k in range(3)])
 rot[:,idx]=Slerp(times,Rotation.from_quat(quats[:,j][:,[1,2,3,0]]))(tt).as_quat()[:,[3,0,1,2]]
out=root.parent/'RoboMimic_Deploy-scalebfm/outputs/reactive_latency_modes'
np.savez_compressed(out/f'direct_{args.clip:03d}_pelvis_hands.npz',fps=50,body_pos_w=pos.astype(np.float32),body_quat_w=rot.astype(np.float32),
 joint_pos=np.zeros((n,29),np.float32),joint_vel=np.zeros((n,29),np.float32),body_lin_vel_w=np.zeros_like(pos,dtype=np.float32),body_ang_vel_w=np.zeros_like(pos,dtype=np.float32),
 joint_names=np.array(meta['joint_names']),body_names=np.array(meta['body_names']))
# Direct vectorized version for only the three requested robot frames.
indices=[NAMES.index(n) for n in selected.values()]
offsets=np.array([solver.pos_offsets1[n] for n in selected.values()])
rotation_offsets=Rotation.from_quat(np.array([solver.rot_offsets1[n].as_quat() for n in selected.values()]))
scales=np.array([solver.human_scale_table[n] for n in selected.values()])[:,None]
fast=[];error=[]
for i in range(len(h['positions'])):
 start=time.perf_counter()
 rootpos=h['positions'][i,0,0]
 rr=Rotation.from_matrix(h['rotations'][i,0,indices])*rotation_offsets
 pp=(h['positions'][i,0,indices]-rootpos)*scales+rootpos*solver.human_scale_table['Hips']+rr.apply(offsets)
 qq=rr.as_quat(scalar_first=True)
 if i>=8:fast.append((time.perf_counter()-start)*1000)
 error.append(float(np.max(np.abs(pp-positions[i]))))
 np.testing.assert_allclose(np.abs(np.sum(qq*quats[i],axis=1)),1,atol=1e-6)
assert max(error)<1e-6
report=dict(direct_vectorized_median_ms=float(np.median(fast)),direct_vectorized_p95_ms=float(np.percentile(fast,95)),max_position_difference_m=max(error),median_ms=float(np.median(samples)),p95_ms=float(np.percentile(samples,95)),samples=len(samples),
 scope='Experimental mode 2 only; unused bodies/joints are placeholders. No IK, no geometry repair, not deployment-approved.')
(out/f'direct_{args.clip:03d}_targets_latency.json').write_text(json.dumps(report,indent=2));print(report)
