"""
Retarget Mixamo BVH motion to humanoid robot.

Why not bvh_to_robot.py:
  The LAFAN1 BVH parser assumes uniform channel counts across all joints.
  Mixamo BVH exports use CHANNELS 6 on many non-root joints, causing a reshape
  error. This script parses per-joint channel counts properly.

Joint name handling:
  Mixamo BVH uses 'mixamorig:Hips' style names. The colon prefix is stripped
  automatically, yielding names like 'Hips', 'LeftArm', etc., which match
  the fbx_offline_to_g1.json IK config.

Usage:
  python scripts/mixamo_bvh_to_robot.py \
    --bvh_file "data/bvh/Stand Up.bvh" \
    --robot unitree_g1 \
    --save_path data/mixamo_out/stand_up.pkl \
    --record_video \
    --video_path videos/stand_up_v2.mp4

"""

import argparse
import os
import re
import pickle
import time

import numpy as np
from scipy.spatial.transform import Rotation as R
from tqdm import tqdm
from rich import print

from general_motion_retargeting import GeneralMotionRetargeting as GMR
from general_motion_retargeting import RobotMotionViewer
import general_motion_retargeting.utils.lafan_vendor.utils as lutils


# ---------------------------------------------------------------------------
# BVH parsing
# ---------------------------------------------------------------------------

def read_mixamo_bvh(filename):
    """
    Parse a Mixamo BVH file. Handles mixed per-joint channel counts (3 or 6).

    Returns
    -------
    lquats  : (T, N, 4)  local quaternions, wxyz
    lpos    : (T, N, 3)  local positions (root animated; others = rest offsets)
    parents : (N,)       parent indices (-1 for root)
    names   : list[str]  joint names with mixamorig: prefix stripped
    frame_time : float   seconds per frame
    """
    with open(filename, "r") as f:
        lines = f.readlines()

    names, offsets_list, parents_list = [], [], []
    joint_chan = []   # per joint: (has_pos: bool, rot_order: str, n_ch: int)
    active = -1
    end_site = False
    in_motion = False
    num_frames = 0
    frame_time = 1.0 / 30.0
    motion_lines = []

    for line in lines:
        line = line.strip()
        if not line:
            continue

        if "MOTION" in line:
            in_motion = True
            continue

        if in_motion:
            if line.startswith("Frames:"):
                num_frames = int(line.split()[1])
            elif line.startswith("Frame Time:"):
                frame_time = float(line.split()[2])
            else:
                motion_lines.append(line)
            continue

        # --- HIERARCHY ---
        m = re.match(r"(?:ROOT|JOINT)\s+(\S+)", line)
        if m:
            raw = m.group(1)
            name = re.sub(r"mixamorig[_:]", "", raw)
            names.append(name)
            offsets_list.append([0.0, 0.0, 0.0])
            parents_list.append(active)
            joint_chan.append(None)
            active = len(parents_list) - 1
            end_site = False
            continue

        if "End Site" in line:
            end_site = True
            continue

        if line.startswith("OFFSET") and not end_site:
            offsets_list[active] = list(map(float, line.split()[1:4]))
            continue

        if line.startswith("CHANNELS") and not end_site:
            parts = line.split()
            n = int(parts[1])
            ch_names = parts[2 : 2 + n]
            has_pos = any("position" in c.lower() for c in ch_names)
            rot_order = "".join(
                c[0].lower() for c in ch_names if "rotation" in c.lower()
            )
            joint_chan[active] = (has_pos, rot_order, n)
            continue

        if "}" in line:
            if end_site:
                end_site = False
            else:
                active = parents_list[active]

    N = len(names)
    T = min(num_frames, len(motion_lines))
    offsets_np = np.array(offsets_list, dtype=float)
    parents_np = np.array(parents_list, dtype=int)

    # local positions: start from rest offsets, root overwritten per frame
    lpos = np.tile(offsets_np[None], (T, 1, 1))
    lrot_euler = np.zeros((T, N, 3))

    for fi, mline in enumerate(motion_lines[:T]):
        values = list(map(float, mline.split()))
        cursor = 0
        for j, (has_pos, _, n_ch) in enumerate(joint_chan):
            if has_pos:
                lpos[fi, j] = values[cursor : cursor + 3]
                lrot_euler[fi, j] = values[cursor + 3 : cursor + 6]
            else:
                lrot_euler[fi, j] = values[cursor : cursor + 3]
            cursor += n_ch

    # euler -> quaternion (wxyz) using per-joint rotation order
    lquats = np.zeros((T, N, 4))
    for j, (_, rot_order, _) in enumerate(joint_chan):
        euler_rad = np.radians(lrot_euler[:, j, :])
        q_xyzw = R.from_euler(rot_order, euler_rad).as_quat()
        lquats[:, j] = q_xyzw[:, [3, 0, 1, 2]]  # xyzw -> wxyz

    return lquats, lpos, parents_np, names, frame_time


def load_mixamo_bvh(bvh_file):
    """
    Load Mixamo BVH and return frames in GMR format.

    Returns
    -------
    frames     : list of dicts  {joint_name: [pos_3d, quat_wxyz]}
    frame_time : float
    """
    lquats, lpos, parents, names, frame_time = read_mixamo_bvh(bvh_file)

    # Forward kinematics -> global rotations and positions
    global_rots, global_pos = lutils.quat_fk(lquats, lpos, parents)

    # y-up (BVH) -> z-up (MuJoCo)
    rot_mat = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], dtype=float)
    rot_q = R.from_matrix(rot_mat).as_quat(scalar_first=True)  # wxyz

    frames = []
    T = lquats.shape[0]
    for fi in range(T):
        frame = {}
        for j, name in enumerate(names):
            orientation = lutils.quat_mul(rot_q, global_rots[fi, j])
            position = global_pos[fi, j] @ rot_mat.T / 100.0  # cm -> m
            frame[name] = [position, orientation]
        frames.append(frame)

    return frames, frame_time


# ---------------------------------------------------------------------------
# Ground offset (same logic as fbx_offline_to_robot.py)
# ---------------------------------------------------------------------------

def compute_ground_offset(retargeter, motion_data):
    minimum = np.inf
    for human_data in motion_data:
        hd = retargeter.to_numpy(human_data)
        hd = retargeter.scale_human_data(
            hd, retargeter.human_root_name, retargeter.human_scale_table
        )
        hd = retargeter.offset_human_data(
            hd, retargeter.pos_offsets1, retargeter.rot_offsets1
        )
        for pos, _ in hd.values():
            if pos[2] < minimum:
                minimum = pos[2]
    return minimum


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Retarget Mixamo BVH to humanoid robot via GMR."
    )
    parser.add_argument("--bvh_file", required=True, help="Path to Mixamo .bvh file")
    parser.add_argument("--robot", default="unitree_g1", help="Target robot name")
    parser.add_argument("--save_path", default=None, help="Output .pkl path")
    parser.add_argument("--rate_limit", action="store_true", default=False)
    parser.add_argument("--record_video", action="store_true", default=False)
    parser.add_argument("--video_path", default="videos/mixamo_example.mp4")
    args = parser.parse_args()

    # Load BVH
    print(f"[bold]Loading:[/bold] {args.bvh_file}")
    frames, frame_time = load_mixamo_bvh(args.bvh_file)
    motion_fps = round(1.0 / frame_time)
    print(f"Loaded {len(frames)} frames @ {motion_fps} FPS")
    print(f"Joints: {list(frames[0].keys())}")

    # Init retargeter (reuses fbx_offline IK config - same joint names as Mixamo)
    retargeter = GMR(
        src_human="bvh_mixamo",
        tgt_robot=args.robot,
        actual_human_height=1.8,
    )
    ground_offset = compute_ground_offset(retargeter, frames)
    retargeter.set_ground_offset(ground_offset)

    if args.save_path:
        os.makedirs(os.path.dirname(os.path.abspath(args.save_path)), exist_ok=True)
        qpos_list = []

    viewer = RobotMotionViewer(
        robot_type=args.robot,
        motion_fps=motion_fps,
        transparent_robot=0,
        record_video=args.record_video,
        video_path=args.video_path,
    )

    fps_counter, fps_start = 0, time.time()
    for frame_data in tqdm(frames, desc="Retargeting"):
        fps_counter += 1
        now = time.time()
        if now - fps_start >= 2.0:
            print(f"FPS: {fps_counter / (now - fps_start):.1f}")
            fps_counter, fps_start = 0, now

        qpos = retargeter.retarget(frame_data)
        viewer.step(
            root_pos=qpos[:3],
            root_rot=qpos[3:7],
            dof_pos=qpos[7:],
            human_motion_data=retargeter.scaled_human_data,
            rate_limit=args.rate_limit,
        )
        if args.save_path:
            qpos_list.append(qpos)

    if args.save_path:
        root_pos = np.array([q[:3] for q in qpos_list])
        root_rot = np.array([q[3:7][[1, 2, 3, 0]] for q in qpos_list])  # wxyz->xyzw
        dof_pos = np.array([q[7:] for q in qpos_list])
        motion_data = {
            "fps": motion_fps,
            "root_pos": root_pos,
            "root_rot": root_rot,
            "dof_pos": dof_pos,
            "local_body_pos": None,
            "link_body_list": None,
        }
        with open(args.save_path, "wb") as f:
            pickle.dump(motion_data, f)
        print(f"[bold green]Saved to {args.save_path}[/bold green]")

    viewer.close()
