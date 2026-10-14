"""Retarget both DuoBox actors with GMR and render synchronized G1 playback.

Uses the existing fbx_offline mapping; no dynamics or inter-robot collision solve.
"""
import argparse
import copy
import json
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

os.environ.setdefault('MUJOCO_GL', 'egl')
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import imageio.v2 as imageio
import mujoco
import numpy as np
from PIL import Image, ImageDraw
from scipy.spatial.transform import Rotation
from general_motion_retargeting import GeneralMotionRetargeting, ROBOT_XML_DICT

NAMES = ['Hips', 'Spine', 'Spine1', 'Neck', 'Head', 'LeftShoulder',
         'LeftArm', 'LeftForeArm', 'LeftHand', 'RightShoulder', 'RightArm',
         'RightForeArm', 'RightHand', 'LeftUpLeg', 'LeftLeg', 'LeftFoot',
         'LeftToeBase', 'RightUpLeg', 'RightLeg', 'RightFoot', 'RightToeBase']
PARENTS = [-1, 0, 1, 2, 3, 2, 5, 6, 7, 2, 9, 10, 11, 0, 13, 14, 15, 0, 17, 18, 19]


def load_pair(path):
    raw = np.load(path, allow_pickle=True).item()
    names = sorted(k[:-9] for k in raw if k.endswith('_position'))
    assert len(names) == 2, 'Expected exactly two actors'
    actors = []
    for name in names:
        pos = np.asarray(raw[name + '_position'][::4], dtype=float)
        rot = np.asarray(raw[name + '_rotation'][::4], dtype=float).copy()
        assert pos.shape[1:] == (21, 3) and rot.shape[1:] == (21, 3, 3)
        # Input root rotation is global; all other rotations are parent-local.
        # DuoBox and MuJoCo both use Z-up here. Do not apply Ready-to-React's Y-up transform.
        for j in range(1, 21):
            rot[:, j] = rot[:, PARENTS[j]] @ rot[:, j]
        quat = Rotation.from_matrix(rot.reshape(-1, 3, 3)).as_quat(scalar_first=True).reshape(-1, 21, 4)
        actors.append((pos, quat))
    length = min(len(a[0]) for a in actors)
    return names, actors, length


def pair_model():
    source = Path(ROBOT_XML_DICT['unitree_g1']).resolve()
    xml = ET.parse(source).getroot()
    xml.find('compiler').set('meshdir', str(source.parent / 'meshes'))
    for tag in ['actuator', 'sensor', 'keyframe']:
        for element in list(xml.findall(tag)):
            xml.remove(element)
    robot_world = next(w for w in xml.findall('worldbody') if w.find('body') is not None)
    original = copy.deepcopy(robot_world.find('body'))
    robot_world.remove(robot_world.find('body'))
    for actor, color in enumerate(['0.15 0.65 0.95 1', '0.95 0.40 0.12 1']):
        robot = copy.deepcopy(original)
        for element in robot.iter():
            if 'name' in element.attrib:
                element.set('name', f'actor{actor}/' + element.get('name'))
            if element.tag == 'geom' and element.get('type') == 'mesh':
                element.set('rgba', color)
        robot_world.append(robot)
    visual = xml.find('visual')
    glob = visual.find('global')
    if glob is None:
        glob = ET.SubElement(visual, 'global')
    glob.set('offwidth', '1280')
    glob.set('offheight', '720')
    return mujoco.MjModel.from_xml_string(ET.tostring(xml, encoding='unicode'))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--continuous', action='store_true', help='Previous-frame regularization and 8 rad/s joint speed cap')
    parser.add_argument('--no-video', action='store_true', help='Save trajectories only, for the interactive browser')
    parser.add_argument('--geometry', action='store_true', help='Arm/torso collision limits and support-foot target repair')
    parser.add_argument("--body-anchor", action="store_true", help="Preserve source pelvis position and pelvis/torso orientation; implies geometry")
    args = parser.parse_args()
    if args.body_anchor:
        args.geometry = True
    if args.geometry:
        args.continuous = True
    args.output.mkdir(parents=True, exist_ok=True)
    if (args.output / 'pair_motion.npz').exists():
        parser.error('Output already contains motion; use a new output directory')
    names, actors, length = load_pair(args.input)
    solutions = []
    diagnostics = {}
    # Same mapping/scaling and one shared floor offset preserve a common scene.
    solvers = [GeneralMotionRetargeting('fbx_offline', 'unitree_g1', actual_human_height=1.8,
                                      solver='daqp', verbose=False) for _ in names]
    common_floor = min(float(a[0][:, [16, 20], 2].min()) for a in actors) * .9
    if args.geometry:
        common_floor = 0.0  # Do not lift the whole clip to compensate for one bad toe sample.
    for solver in solvers:
        solver.set_ground_offset(common_floor)
    for name, (pos, quat), solver in zip(names, actors, solvers):
        qs = []
        if args.continuous:
            from duobox_continuity import add_continuity, solve_frame
            continuity, posture = add_continuity(solver)
        geometry = None
        if args.geometry:
            from duobox_geometry import GeometryRepair
            geometry = GeometryRepair(solver, body_anchor=args.body_anchor)
        clearances = []
        for frame in range(length):
            target = {joint: (pos[frame, j].copy(), quat[frame, j].copy()) for j, joint in enumerate(NAMES)}
            q = solve_frame(solver, target, geometry) if args.continuous else solver.retarget(target).copy()
            assert np.isfinite(q).all()
            qs.append(q)
            if geometry is not None:
                clearances.append(geometry.minimum_distance(solver))
            if args.continuous:
                continuity.previous = q.copy()
                posture.set_target(q)
            if frame % 100 == 0:
                print(f'{name}: {frame}/{length}', flush=True)
        qs = np.asarray(qs)
        violations = 0.
        for j in range(solver.model.njnt):
            if solver.model.jnt_limited[j]:
                values = qs[:, solver.model.jnt_qposadr[j]]
                lo, hi = solver.model.jnt_range[j]
                violations = max(violations, float(np.maximum(lo-values, values-hi).max()))
        diagnostics[name] = {'max_joint_limit_violation_rad': violations,
                             'max_frame_joint_change_rad': float(np.abs(np.diff(qs[:, 7:], axis=0)).max())}
        if args.continuous:
            assert diagnostics[name]['max_frame_joint_change_rad'] <= 8 / 30 + 1e-5
        if clearances:
            diagnostics[name]['min_arm_torso_clearance_m'] = min(clearances)
        solutions.append(qs)
    qpos = np.stack(solutions, axis=1)
    np.savez_compressed(args.output / 'pair_motion.npz', qpos=qpos, fps=30, actors=np.array(names))
    meta = {'source': str(args.input.resolve()), 'actors': names, 'frames': length, 'fps': 30,
            'seconds': length / 30, 'method': 'GMR fbx_offline -> unitree_g1; independent IK',
            'quaternion': 'wxyz', 'shared_ground_offset': common_floor,
            'root_scale': .9, 'arm_scale': .8, 'dynamics': False, 'diagnostics': diagnostics}
    meta['continuity'] = {'enabled': args.continuous, 'joint_speed_cap_rad_s': 8.0 if args.continuous else None,
                          'posture_cost': .2 if args.continuous else 0,
                          'floating_base_limited': False, 'acceleration_limited': False}
    meta['geometry'] = {'enabled': args.geometry, 'source_ground_z': 0.0,
                        'foot_contact': 'height/speed heuristic', 'pair_collision': False, 'revision': 10 if args.body_anchor else 7, 'body_anchor': args.body_anchor,
                        'hand_frame_budget': '2cm + 1.5x source displacement; 8deg + 1.5x min(source rotation, atan2(source displacement, 0.3m))' if args.geometry else None}
    (args.output / 'metadata.json').write_text(json.dumps(meta, indent=2))
    if args.no_video:
        print(json.dumps(meta, indent=2), flush=True)
        return
    model = pair_model()
    assert model.nq == qpos.shape[1] * qpos.shape[2]
    data = mujoco.MjData(model)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = [*qpos[:, :, :2].mean(axis=(0, 1)), .85]
    camera.distance = 3.8
    camera.azimuth = 130
    camera.elevation = -18
    option = mujoco.MjvOption()
    option.geomgroup[3:] = 0
    with mujoco.Renderer(model, height=720, width=1280) as renderer:
        with imageio.get_writer(args.output / 'g1_pair.mp4', fps=30, codec='libx264', quality=8) as writer:
            for frame in range(length):
                data.qpos[:] = qpos[frame].reshape(-1)
                mujoco.mj_forward(model, data)
                renderer.update_scene(data, camera, scene_option=option)
                image = Image.fromarray(renderer.render())
                draw = ImageDraw.Draw(image)
                draw.rectangle((0, 0, 1280, 53), fill=(20, 25, 35))
                draw.text((16, 10), f'DuoBox -> GMR -> G1 | blue: {names[0]} | orange: {names[1]} | {frame/30:.2f}s / {length/30:.2f}s', fill='white')
                draw.text((16, 30), 'Kinematic replay | independent IK | no dynamics / inter-robot collision constraints', fill='white')
                writer.append_data(np.asarray(image))
                if frame == min(90, length - 1):
                    image.save(args.output / 'preview.jpg')
    (args.output / 'index.html').write_text('''<!doctype html><meta charset="utf-8"><title>G1 双人拳击重定向回放</title>
<body style="background:#111827;color:white;font:18px system-ui;margin:30px"><h2>G1 双人拳击 · GMR 重定向</h2>
<p>004 · 直拳与闪避 · 原始 DuoBox 动作重定向 · 双方同步</p>
<video controls autoplay muted loop style="width:100%;max-height:75vh" src="g1_pair.mp4"></video>
<p>运动学回放：没有物理平衡或双机器人碰撞约束。两人的根轨迹共同缩放至 0.9 倍。</p>
<a href="g1_pair.mp4" style="color:#38bdf8">下载视频</a> · <a href="pair_motion.npz" style="color:#38bdf8">下载双人关节轨迹</a></body>''')
    print(json.dumps(meta, indent=2), flush=True)


if __name__ == '__main__':
    main()
