# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Installation and Setup

```bash
conda create -n gmr python=3.10 -y
conda activate gmr
pip install -e .
conda install -c conda-forge libstdcxx-ng -y  # fixes rendering issues
```

After installing, change `ext` in `smplx/body_models.py` from `npz` to `pkl` if using SMPL-X pkl files.

SMPL-X body models must be placed at:
```
assets/body_models/smplx/SMPLX_NEUTRAL.pkl
assets/body_models/smplx/SMPLX_FEMALE.pkl
assets/body_models/smplx/SMPLX_MALE.pkl
```

## Common Commands

```bash
# SMPL-X to robot (single)
python scripts/smplx_to_robot.py --smplx_file <path> --robot <robot_name> --save_path <output.pkl> --rate_limit

# BVH to robot (single) - --format options: lafan1, nokov
python scripts/bvh_to_robot.py --bvh_file <path> --robot <robot_name> --save_path <output.pkl> --rate_limit --format <format>

# Xsens BVH to robot (requires running mujoco_xsens_bvh_view.py first to generate offset.json)
python scripts/xsens_bvh_to_robot.py --bvh_file <path> --robot <robot_name> --save_path <output.pkl> --scale 0.01 --reset_to_zero --bvh_format 3DSM

# FBX offline to robot
python scripts/fbx_offline_to_robot.py --motion_file <path> --robot <robot_name> --save_path <output.pkl>

# GVHMR (monocular video) to robot
python scripts/gvhmr_to_robot.py --gvhmr_pred_file <path/hmr4d_results.pt> --robot <robot_name>

# OptiTrack real-time streaming
python scripts/optitrack_to_robot.py --server_ip <ip> --client_ip <ip> --robot unitree_g1

# Batch processing
python scripts/smplx_to_robot_dataset.py --src_folder <dir> --tgt_folder <dir> --robot <robot_name>
python scripts/bvh_to_robot_dataset.py --src_folder <dir> --tgt_folder <dir> --robot <robot_name>

# Visualize saved robot motion
python scripts/vis_robot_motion.py --robot <robot_name> --robot_motion_path <path.pkl>
python scripts/vis_robot_motion_dataset.py --robot <robot_name> --robot_motion_folder <dir>
# Add --record_video --video_path <output.mp4> to record

# Visualize Xsens BVH in MuJoCo (also generates offset.json for retargeting)
python general_motion_retargeting/utils/xsens_vendor/mujoco_xsens_bvh_view.py --bvh_file <path> --scale 0.01 --reset_to_zero
```

## Code Architecture

### Core Components

- **`GeneralMotionRetargeting`** (`general_motion_retargeting/motion_retarget.py`): Main retargeting class. Instantiated with `src_human` (data format key) and `tgt_robot` (robot key). Loads the MuJoCo XML model and IK config, sets up mink `FrameTask` objects, then solves IK frame-by-frame.
- **`KinematicsModel`** (`general_motion_retargeting/kinematics_model.py`): Robot kinematics utilities.
- **`RobotMotionViewer`** (`general_motion_retargeting/robot_motion_viewer.py`): MuJoCo-based viewer with optional video recording.
- **`params.py`** (`general_motion_retargeting/params.py`): Central registry — `ROBOT_XML_DICT` maps robot names to MuJoCo XML paths, `IK_CONFIG_DICT` maps `(src_format, robot_name)` to JSON config paths, `ROBOT_BASE_DICT` maps robot names to their root body name.

### Data Flow

1. **Input**: Human motion loaded as a list of per-frame dicts: `{body_name: (3D_translation, quaternion_wxyz)}`
2. **Scaling**: Human joint positions scaled by `human_scale_table` (per-body scale factors), optionally adjusted by `actual_human_height / human_height_assumption` ratio
3. **IK**: Two-phase mink IK solve (`ik_match_table1` then `ik_match_table2`) — each table entry maps a robot body to a human body with position/rotation weights and offsets
4. **Output**: Pickle file with keys: `fps`, `root_pos`, `root_rot` (xyzw), `dof_pos`, `local_body_pos`, `link_body_list`

### IK Config JSON Structure

Each `ik_configs/*.json` file defines the human-to-robot mapping:

```json
{
  "robot_root_name": "pelvis",       // robot base body name (must match ROBOT_BASE_DICT)
  "human_root_name": "pelvis",       // source skeleton root joint name
  "ground_height": 0.0,
  "human_height_assumption": 1.8,    // assumed human height in meters for scaling
  "use_ik_match_table1": true,
  "use_ik_match_table2": true,
  "human_scale_table": {
    "pelvis": 0.9,                   // scale factor per human body part
    ...
  },
  "ik_match_table1": {
    "robot_body_name": [
      "human_body_name",
      100,                           // position tracking weight
      10,                            // rotation tracking weight
      [0.0, 0.0, 0.0],              // position offset (xyz)
      [0.5, -0.5, -0.5, -0.5]      // rotation offset (quaternion wxyz)
    ],
    ...
  },
  "ik_match_table2": { ... }        // second IK pass, same format
}
```

### Supported Robots (from `params.py`)

| Robot name | DOF |
|---|---|
| `unitree_g1` | 29 |
| `unitree_g1_with_hands` | 43 |
| `unitree_h1` | 19 |
| `unitree_h1_2` | 27 |
| `booster_t1` | — |
| `booster_t1_29dof` | 29 |
| `booster_k1` | 22 |
| `stanford_toddy` | — |
| `fourier_n1` | — |
| `fourier_gr3` | 31 |
| `engineai_pm01` | — |
| `kuavo_s45` | 28 |
| `hightorque_hi` | 25 |
| `galaxea_r1pro` | 24 (wheeled) |
| `berkeley_humanoid_lite` | 22 |
| `pnd_adam_lite` | 25 |
| `tienkung` | 20 |
| `pal_talos` | 30 |

### Adding a New Robot

1. Add MuJoCo XML to `assets/<robot_name>/`
2. Register in `params.py`: `ROBOT_XML_DICT`, `ROBOT_BASE_DICT`, `IK_CONFIG_DICT` (for each supported source format), `VIEWER_CAM_DISTANCE_DICT`
3. Create IK config JSON in `general_motion_retargeting/ik_configs/`

### Key Technical Details

- **IK solver**: mink with default solver `"daqp"`, damping `5e-1`, max 10 iterations per frame
- **Quaternion convention**: `wxyz` throughout (MuJoCo convention); output pickle stores `root_rot` as `xyzw`
- **Velocity limits**: `use_velocity_limit=True` caps joint velocity at `3*pi` rad/s per motor
- **`GeneralMotionRetargeting` attributes**: `robot_dof_names`, `robot_body_names`, `robot_motor_names` dicts are populated on init for downstream use
- **Third-party**: `third_party/poselib/` provides FBX import and rotation utilities; `general_motion_retargeting/optitrack_vendor/` provides NatNet client for real-time OptiTrack streaming
