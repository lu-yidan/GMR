"""
可视化调试 IK retargeting 效果。

显示内容：
  - 左侧：机器人当前姿态（IK 求解结果）
  - 右侧：human keypoints（IK target，经过 scale/offset 变换后）带关节名标签
  - 红线：IK target（人体关节）→ 机器人实际 body 位置，线越长误差越大
  - 终端：每帧各关节误差数值，方便定位哪个关节需要调参数

控制：
  空格键 = 暂停/继续
  Q 键   = 退出

典型用法（Mixamo BVH）：
  python scripts/debug_retarget.py \\
    --bvh_file "data/bvh/Standing Up.bvh" \\
    --src_format mixamo \\
    --robot unitree_g1

典型用法（LAFAN1 BVH）：
  python scripts/debug_retarget.py \\
    --bvh_file data/lafan1/walk1_subject5.bvh \\
    --src_format lafan1 \\
    --robot unitree_g1

IK config 调参指南（general_motion_retargeting/ik_configs/*.json）：
  - pos_weight / rot_weight：增大 = 更努力追踪该关节，减小 = 放宽限制
  - pos_offset [x,y,z]：关节位置偏移（局部坐标系）
  - rot_offset [w,x,y,z]：关节旋转偏移（调整 human 坐标系与 robot 坐标系的对齐）
  误差大的关节优先调整 rot_offset
"""

import argparse
import sys
import time

import mujoco as mj
import numpy as np
from rich import print
from rich.table import Table
from rich.console import Console
from scipy.spatial.transform import Rotation as R

from general_motion_retargeting import GeneralMotionRetargeting as GMR
from general_motion_retargeting import RobotMotionViewer
from general_motion_retargeting.utils.lafan1 import load_bvh_file

sys.path.insert(0, ".")
from scripts.mixamo_bvh_to_robot import load_mixamo_bvh, compute_ground_offset

console = Console()

# ─────────────────────────────────────────────
# 绘制辅助函数
# ─────────────────────────────────────────────

def draw_sphere(viewer, pos, radius=0.03, rgba=(0.2, 0.8, 0.2, 0.9), label=""):
    if viewer.user_scn.ngeom >= viewer.user_scn.maxgeom - 1:
        return
    geom = viewer.user_scn.geoms[viewer.user_scn.ngeom]
    mj.mjv_initGeom(
        geom,
        type=mj.mjtGeom.mjGEOM_SPHERE,
        size=[radius, radius, radius],
        pos=np.array(pos, dtype=float),
        mat=np.eye(3).flatten(),
        rgba=np.array(rgba, dtype=float),
    )
    geom.label = label
    viewer.user_scn.ngeom += 1


def draw_line(viewer, from_pos, to_pos, rgba=(1.0, 0.2, 0.2, 0.8), width=0.004):
    if viewer.user_scn.ngeom >= viewer.user_scn.maxgeom - 1:
        return
    mj.mjv_connector(
        viewer.user_scn.geoms[viewer.user_scn.ngeom],
        type=mj.mjtGeom.mjGEOM_CAPSULE,
        width=width,
        from_=np.array(from_pos, dtype=float),
        to=np.array(to_pos, dtype=float),
    )
    viewer.user_scn.geoms[viewer.user_scn.ngeom].rgba = np.array(rgba, dtype=float)
    viewer.user_scn.ngeom += 1


def draw_frame_axes(viewer, pos, rot_mat, size=0.08, label=""):
    """在给定位置绘制 xyz 坐标轴 (红=x, 绿=y, 蓝=z)"""
    colors = [(1, 0, 0, 1), (0, 1, 0, 1), (0, 0, 1, 1)]
    for i, rgba in enumerate(colors):
        if viewer.user_scn.ngeom >= viewer.user_scn.maxgeom - 1:
            return
        tip = np.array(pos) + size * rot_mat[:, i]
        mj.mjv_connector(
            viewer.user_scn.geoms[viewer.user_scn.ngeom],
            type=mj.mjtGeom.mjGEOM_ARROW,
            width=0.005,
            from_=np.array(pos, dtype=float),
            to=tip.astype(float),
        )
        viewer.user_scn.geoms[viewer.user_scn.ngeom].rgba = np.array(rgba, dtype=float)
        if i == 0:
            viewer.user_scn.geoms[viewer.user_scn.ngeom].label = label
        viewer.user_scn.ngeom += 1


# ─────────────────────────────────────────────
# 误差打印
# ─────────────────────────────────────────────

def print_errors(retargeter, frame_idx):
    """打印每个 IK 任务的位置/旋转误差"""
    table = Table(title=f"Frame {frame_idx}  IK errors", show_lines=False)
    table.add_column("Robot body", style="cyan", min_width=28)
    table.add_column("Human joint", style="yellow", min_width=20)
    table.add_column("Pos err (m)", justify="right", min_width=12)
    table.add_column("Rot err (rad)", justify="right", min_width=14)

    all_tasks = []
    for human_name, task in retargeter.human_body_to_task1.items():
        all_tasks.append((human_name, task, "T1"))
    for human_name, task in retargeter.human_body_to_task2.items():
        all_tasks.append((human_name, task, "T2"))

    rows = []
    for human_name, task, tbl in all_tasks:
        err = task.compute_error(retargeter.configuration)
        pos_err = float(np.linalg.norm(err[:3]))
        rot_err = float(np.linalg.norm(err[3:]))
        rows.append((task.frame_name, human_name, pos_err, rot_err, tbl))

    rows.sort(key=lambda r: r[2] + r[3], reverse=True)
    for robot_name, human_name, pos_err, rot_err, tbl in rows:
        style = "bold red" if pos_err > 0.1 or rot_err > 0.5 else ""
        table.add_row(
            f"{robot_name} ({tbl})",
            human_name,
            f"{pos_err:.4f}",
            f"{rot_err:.4f}",
            style=style,
        )
    console.print(table)


# ─────────────────────────────────────────────
# 主逻辑
# ─────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="可视化调试 IK retargeting")
    parser.add_argument("--bvh_file", required=True)
    parser.add_argument(
        "--src_format",
        choices=["mixamo", "lafan1", "nokov"],
        default="mixamo",
        help="mixamo: Mixamo BVH；lafan1/nokov: 对应格式 BVH",
    )
    parser.add_argument("--robot", default="unitree_g1")
    parser.add_argument(
        "--human_offset",
        type=float,
        default=1.5,
        help="human keypoints 向右偏移距离 (m)，与机器人并排显示",
    )
    parser.add_argument(
        "--print_errors",
        action="store_true",
        default=False,
        help="每帧在终端打印各关节误差（会降低速度）",
    )
    parser.add_argument(
        "--print_every",
        type=int,
        default=30,
        help="每隔多少帧打印一次误差（默认 30）",
    )
    parser.add_argument("--rate_limit", action="store_true", default=False)
    parser.add_argument(
        "--actual_human_height",
        type=float,
        default=1.8,
        help="实际人体身高(m)，config 假设 1.8m。G1 约 0.79m，建议设 1.0~1.3 缩小 human keypoints",
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=None,
        help="手动指定播放帧率，如 5 = 慢放，默认跟随原始帧率",
    )
    args = parser.parse_args()

    # 加载数据
    if args.src_format == "mixamo":
        frames, frame_time = load_mixamo_bvh(args.bvh_file)
        motion_fps = round(1.0 / frame_time)
        src_human = "bvh_mixamo"
    else:
        frames, _ = load_bvh_file(args.bvh_file, format=args.src_format)
        motion_fps = 30
        src_human = f"bvh_{args.src_format}"

    print(f"加载完成：{len(frames)} 帧  @{motion_fps} FPS  关节数：{len(frames[0])}")

    # 初始化 retargeter
    retargeter = GMR(src_human=src_human, tgt_robot=args.robot, actual_human_height=args.actual_human_height)
    ground_offset = compute_ground_offset(retargeter, frames)
    retargeter.set_ground_offset(ground_offset)

    # 初始化 viewer
    viewer = RobotMotionViewer(
        robot_type=args.robot,
        motion_fps=motion_fps,
        transparent_robot=1,
        record_video=False,
    )

    # 状态
    paused = False
    frame_idx = 0
    human_offset = np.array([args.human_offset, 0.0, 0.0])

    print(f"\n[bold green]控制说明[/bold green]")
    print(f"  空格键  = 暂停/继续")
    print(f"  Q      = 退出")
    print(f"  human keypoints 偏移 {args.human_offset}m 显示在机器人右侧\n")

    # 键盘回调
    def key_callback(keycode):
        nonlocal paused
        if keycode == ord(" "):
            paused = not paused
            print(f"{'[暂停]' if paused else '[继续]'}")

    viewer.viewer._key_callback = key_callback  # 直接绑定

    from loop_rate_limiters import RateLimiter
    playback_fps = args.fps if args.fps is not None else motion_fps
    rate_limiter = RateLimiter(frequency=playback_fps, warn=False)
    print(f"播放帧率：{playback_fps} FPS（原始：{motion_fps} FPS）\n")

    while viewer.viewer.is_running():
        if paused:
            viewer.viewer.sync()
            time.sleep(0.05)
            continue

        frame_data = frames[frame_idx]
        qpos = retargeter.retarget(frame_data)
        scaled = retargeter.scaled_human_data  # IK target (变换后的 human keypoints)

        # 更新机器人姿态
        viewer.data.qpos[:3] = qpos[:3]
        viewer.data.qpos[3:7] = qpos[3:7]
        viewer.data.qpos[7:] = qpos[7:]
        mj.mj_forward(viewer.model, viewer.data)

        # 相机跟随机器人
        from general_motion_retargeting import ROBOT_BASE_DICT
        base_id = viewer.model.body(ROBOT_BASE_DICT[args.robot]).id
        viewer.viewer.cam.lookat = viewer.data.xpos[base_id]
        viewer.viewer.cam.distance = 3.0
        viewer.viewer.cam.elevation = -15

        # ── 绘制自定义几何体 ──────────────────────────────
        viewer.viewer.user_scn.ngeom = 0

        # 1. Human keypoints：带标签的坐标轴 + 绿色球（偏移到右侧）
        for human_name, (pos, quat) in scaled.items():
            h_pos = pos + human_offset
            rot_mat = R.from_quat(quat, scalar_first=True).as_matrix()
            draw_frame_axes(viewer.viewer, h_pos, rot_mat, size=0.07, label=human_name)
            draw_sphere(viewer.viewer, h_pos, radius=0.025,
                       rgba=(0.2, 0.9, 0.2, 0.9))

        # 2. 机器人 body 位置：黄色球（只显示被 IK track 的 body）
        all_tasks = list(retargeter.human_body_to_task1.items()) + \
                    list(retargeter.human_body_to_task2.items())
        robot_body_pos = {}
        for human_name, task in all_tasks:
            robot_name = task.frame_name
            if robot_name not in robot_body_pos:
                body_id = viewer.model.body(robot_name).id
                robot_body_pos[robot_name] = viewer.data.xpos[body_id].copy()
                draw_sphere(viewer.viewer, robot_body_pos[robot_name],
                           radius=0.025, rgba=(1.0, 0.9, 0.0, 0.9), label=robot_name)

        # 3. 连线：IK target → 机器人实际位置（红线，误差越大越长）
        drawn_pairs = set()
        for human_name, task in all_tasks:
            robot_name = task.frame_name
            if (human_name, robot_name) in drawn_pairs:
                continue
            drawn_pairs.add((human_name, robot_name))
            if human_name in scaled and robot_name in robot_body_pos:
                h_pos = scaled[human_name][0] + human_offset
                r_pos = robot_body_pos[robot_name]
                err_len = float(np.linalg.norm(h_pos - r_pos))
                # 误差 > 3cm 才画线，避免太乱
                if err_len > 0.03:
                    # 红色深浅代表误差大小
                    intensity = min(err_len / 0.3, 1.0)
                    draw_line(viewer.viewer, h_pos, r_pos,
                             rgba=(intensity, 1.0 - intensity * 0.8, 0.1, 0.8))

        viewer.viewer.sync()
        if args.rate_limit or args.fps is not None:
            rate_limiter.sleep()

        # 终端打印误差
        if args.print_errors and frame_idx % args.print_every == 0:
            print_errors(retargeter, frame_idx)

        frame_idx = (frame_idx + 1) % len(frames)

    viewer.close()


if __name__ == "__main__":
    main()
