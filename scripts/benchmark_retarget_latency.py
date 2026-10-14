"""Measure warm single-actor retarget and direct robot body-pose extraction."""
import argparse,json,time
from pathlib import Path
import mujoco,numpy as np
from scipy.spatial.transform import Rotation
from duobox_to_g1_pair import NAMES,GeneralMotionRetargeting
from duobox_continuity import add_continuity,solve_frame
from duobox_geometry import GeometryRepair
p=argparse.ArgumentParser();p.add_argument('--human',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
h=dict(np.load(a.human));scale=Path(__file__).resolve().parents[2]/'ScaleBFM'
meta=json.loads((scale/'artifacts/scalebfm/humanoid_transformer_m/model_22200_tensorrt_metadata.json').read_text())
m=mujoco.MjModel.from_xml_path(str(scale/'ScaleBridge/scalebridge/data/robot/g1_29dof/g1_29dof.xml'));d=mujoco.MjData(m)
selected=[m.body(n).id for n in meta['selected_body_names']]
report={}
def stats(x):return dict(median_ms=float(np.median(x)),p95_ms=float(np.percentile(x,95)),max_ms=float(np.max(x)),samples=len(x))
for mode in ['v10','original_gmr']:
 solver=GeneralMotionRetargeting('fbx_offline','unitree_g1',actual_human_height=1.8,solver='daqp',verbose=False);solver.set_ground_offset(0.)
 ids=[solver.model.body(n).id for n in meta['selected_body_names']]
 source=[solver.model.joint(n).qposadr[0] for n in meta['joint_names']];target=[m.joint(n).qposadr[0] for n in meta['joint_names']]
 if mode=='v10': continuity,posture=add_continuity(solver);geometry=GeometryRepair(solver,body_anchor=True)
 lat=[];extract=[];fk=[];pos_error=[];rot_error=[];qs=[]
 for i in range(len(h['positions'])):
  start=time.perf_counter();quat=Rotation.from_matrix(h['rotations'][i,0]).as_quat(scalar_first=True)
  human={n:(h['positions'][i,0,j].copy(),quat[j].copy()) for j,n in enumerate(NAMES)}
  q=solve_frame(solver,human,geometry) if mode=='v10' else solver.retarget(human).copy()
  if mode=='v10':continuity.previous=q.copy();posture.set_target(q)
  end=time.perf_counter();bodypos=solver.configuration.data.xpos[ids].copy();bodyquat=solver.configuration.data.xquat[ids].copy();end2=time.perf_counter()
  d.qpos[:7]=q[:7];d.qpos[target]=q[source];mujoco.mj_forward(m,d);end3=time.perf_counter()
  if i>=8:
   lat.append((end-start)*1000);extract.append((end2-end)*1000);fk.append((end3-end2)*1000)
  pos_error.append(np.max(np.linalg.norm(bodypos-d.xpos[selected],axis=1)))
  rot_error.append(np.max(2*np.arccos(np.clip(np.abs(np.sum(bodyquat*d.xquat[selected],axis=1)),0,1))))
  qs.append(q)
 report[mode]=dict(retarget=stats(lat),extract_14_poses=stats(extract),extra_official_fk=stats(fk),
  max_cross_xml_position_error_m=max(pos_error),max_cross_xml_orientation_error_rad=max(rot_error),
  max_joint_frame_delta_rad=float(np.max(np.abs(np.diff(np.array(qs)[:,7:],axis=0)))))
a.out.write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
