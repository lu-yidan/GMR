import argparse
import pathlib
import os
import time

import numpy as np

from general_motion_retargeting import GeneralMotionRetargeting as GMR
from general_motion_retargeting import RobotMotionViewer
from general_motion_retargeting.utils.smpl import load_smplx_file, get_smplx_data_offline_fast

from rich import print

if __name__ == "__main__":
    
    HERE = pathlib.Path(__file__).parent

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--smplx_file",
        help="SMPLX motion file to load.",
        type=str,
        # required=True,
        default="/home/yanjieze/projects/g1_wbc/GMR/motion_data/ACCAD/Male1General_c3d/General_A1_-_Stand_stageii.npz",
        # default="/home/yanjieze/projects/g1_wbc/GMR/motion_data/ACCAD/Male2MartialArtsKicks_c3d/G8_-__roundhouse_left_stageii.npz"
        # default="/home/yanjieze/projects/g1_wbc/TWIST-dev/motion_data/AMASS/KIT_572_dance_chacha11_stageii.npz"
        # default="/home/yanjieze/projects/g1_wbc/GMR/motion_data/ACCAD/Male2MartialArtsPunches_c3d/E1_-__Jab_left_stageii.npz",
        # default="/home/yanjieze/projects/g1_wbc/GMR/motion_data/ACCAD/Male1Running_c3d/Run_C24_-_quick_side_step_left_stageii.npz",
    )
    
    parser.add_argument(
        "--robot",
        choices=["unitree_g1", "unitree_g1_with_hands", "unitree_h1", "unitree_h1_2",
                 "booster_t1", "booster_t1_29dof","stanford_toddy", "fourier_n1", 
                "engineai_pm01", "kuavo_s45", "hightorque_hi", "galaxea_r1pro", "berkeley_humanoid_lite", "booster_k1",
                "pnd_adam_lite", "openlong"],
        default="openlong",
    )
    
    parser.add_argument(
        "--save_path",
        default=None,
        help="Path to save the robot motion.",
    )
    
    parser.add_argument(
        "--loop",
        default=False,
        action="store_true",
        help="Loop the motion.",
    )

    parser.add_argument(
        "--record_video",
        default=False,
        action="store_true",
        help="Record the video.",
    )

    parser.add_argument(
        "--rate_limit",
        default=False,
        action="store_true",
        help="Limit the rate of the retargeted robot motion to keep the same as the human motion.",
    )

    parser.add_argument(
        "--video_width",
        type=int,
        default=640,
        help="Video width in pixels.",
    )

    parser.add_argument(
        "--video_height", 
        type=int,
        default=480,
        help="Video height in pixels.",
    )

    parser.add_argument(
        "--video_fps",
        type=int,
        default=None,
        help="Target video FPS. If higher than motion FPS, motion will be interpolated.",
    )

    args = parser.parse_args()


    SMPLX_FOLDER = HERE / ".." / "assets" / "body_models"
    
    
    # Load SMPLX trajectory
    smplx_data, body_model, smplx_output, actual_human_height = load_smplx_file(
        args.smplx_file, SMPLX_FOLDER
    )
    
    # align fps
    tgt_fps = 30
    smplx_data_frames, aligned_fps = get_smplx_data_offline_fast(smplx_data, body_model, smplx_output, tgt_fps=tgt_fps)
    
   
    # Initialize the retargeting system
    retarget = GMR(
        actual_human_height=actual_human_height,
        src_human="smplx",
        tgt_robot=args.robot,
    )
    
    # Handle video FPS interpolation
    if args.video_fps is not None and args.video_fps > aligned_fps:
        # Interpolate motion data to higher FPS
        print(f"Interpolating motion from {aligned_fps} FPS to {args.video_fps} FPS")
        from scipy.interpolate import interp1d
        
        # Create time arrays
        original_time = np.arange(len(smplx_data_frames)) / aligned_fps
        target_time = np.arange(int(len(smplx_data_frames) * args.video_fps / aligned_fps)) / args.video_fps
        
        # Get the keys from the first frame
        frame_keys = list(smplx_data_frames[0].keys())
        print(f"Frame keys: {frame_keys}")
        
        # Interpolate each motion component
        interpolated_data = {}
        for key in frame_keys:
            data = smplx_data_frames[0][key]
            
            # Handle different data types
            if isinstance(data, (list, tuple)):
                # For list/tuple data (e.g., [pos, rot] pairs)
                if len(data) == 2 and isinstance(data[0], np.ndarray) and isinstance(data[1], np.ndarray):
                    # This is likely a [position, rotation] pair
                    pos_data = np.array([frame[key][0] for frame in smplx_data_frames])
                    rot_data = np.array([frame[key][1] for frame in smplx_data_frames])
                    
                    # Interpolate position and rotation separately
                    pos_interpolator = interp1d(original_time, pos_data, axis=0, kind='linear', fill_value='extrapolate')
                    rot_interpolator = interp1d(original_time, rot_data, axis=0, kind='linear', fill_value='extrapolate')
                    
                    interpolated_data[key] = [
                        pos_interpolator(target_time),
                        rot_interpolator(target_time)
                    ]
                else:
                    # For other list/tuple types, try to convert to numpy array first
                    try:
                        array_data = np.array([frame[key] for frame in smplx_data_frames])
                        if array_data.ndim >= 1:
                            interpolator = interp1d(original_time, array_data, axis=0, kind='linear', fill_value='extrapolate')
                            interpolated_data[key] = interpolator(target_time)
                        else:
                            # Scalar data
                            interpolated_data[key] = np.full(len(target_time), data)
                    except:
                        # If conversion fails, just repeat the data
                        interpolated_data[key] = [data] * len(target_time)
                        
            elif isinstance(data, np.ndarray):
                # For numpy array data
                if data.ndim >= 1:
                    interpolator = interp1d(original_time, np.array([frame[key] for frame in smplx_data_frames]), axis=0, kind='linear', fill_value='extrapolate')
                    interpolated_data[key] = interpolator(target_time)
                else:
                    # Scalar array
                    interpolated_data[key] = np.full(len(target_time), data)
            else:
                # For scalar data, repeat the value
                interpolated_data[key] = [data] * len(target_time)
        
        # Reconstruct interpolated data
        smplx_data_frames = []
        for t in range(len(target_time)):
            frame_data = {}
            for key in frame_keys:
                if isinstance(interpolated_data[key], (list, tuple)):
                    if len(interpolated_data[key]) == 2 and isinstance(interpolated_data[key][0], np.ndarray):
                        # Handle [pos, rot] pairs
                        frame_data[key] = [
                            interpolated_data[key][0][t],
                            interpolated_data[key][1][t]
                        ]
                    else:
                        # Handle other list types
                        frame_data[key] = interpolated_data[key][t]
                elif isinstance(interpolated_data[key], np.ndarray):
                    frame_data[key] = interpolated_data[key][t]
                else:
                    frame_data[key] = interpolated_data[key][t]
            smplx_data_frames.append(frame_data)
        
        aligned_fps = args.video_fps
        print(f"Interpolation complete. New frame count: {len(smplx_data_frames)}")
    
    video_fps = args.video_fps if args.video_fps is not None else aligned_fps
    
    robot_motion_viewer = RobotMotionViewer(robot_type=args.robot,
                                            motion_fps=aligned_fps,
                                            transparent_robot=0,
                                            record_video=args.record_video,
                                            video_path=f"videos/{args.robot}_{args.smplx_file.split('/')[-1].split('.')[0]}.mp4",
                                            video_width=args.video_width,
                                            video_height=args.video_height,
                                            video_fps=video_fps)
    

    curr_frame = 0
    # FPS measurement variables
    fps_counter = 0
    fps_start_time = time.time()
    fps_display_interval = 2.0  # Display FPS every 2 seconds
    
    if args.save_path is not None:
        save_dir = os.path.dirname(args.save_path)
        if save_dir:  # Only create directory if it's not empty
            os.makedirs(save_dir, exist_ok=True)
        qpos_list = []
    
    # Start the viewer
    i = 0

    while True:
        if args.loop:
            i = (i + 1) % len(smplx_data_frames)
        else:
            i += 1
            if i >= len(smplx_data_frames):
                break
        
        # FPS measurement
        fps_counter += 1
        current_time = time.time()
        if current_time - fps_start_time >= fps_display_interval:
            actual_fps = fps_counter / (current_time - fps_start_time)
            print(f"Actual rendering FPS: {actual_fps:.2f}")
            fps_counter = 0
            fps_start_time = current_time
        
        # Update task targets.
        smplx_data = smplx_data_frames[i]

        # retarget
        qpos = retarget.retarget(smplx_data)

        # visualize
        robot_motion_viewer.step(
            root_pos=qpos[:3],
            root_rot=qpos[3:7],
            dof_pos=qpos[7:],
            human_motion_data=retarget.scaled_human_data,
            # human_motion_data=smplx_data,
            human_pos_offset=np.array([0.0, 0.0, 0.0]),
            show_human_body_name=False,
            rate_limit=args.rate_limit,
        )
        if args.save_path is not None:
            qpos_list.append(qpos)
            
    if args.save_path is not None:
        import pickle
        root_pos = np.array([qpos[:3] for qpos in qpos_list])
        # save from wxyz to xyzw
        root_rot = np.array([qpos[3:7][[1,2,3,0]] for qpos in qpos_list])
        dof_pos = np.array([qpos[7:] for qpos in qpos_list])
        local_body_pos = None
        body_names = None
        
        motion_data = {
            "fps": aligned_fps,
            "root_pos": root_pos,
            "root_rot": root_rot,
            "dof_pos": dof_pos,
            "local_body_pos": local_body_pos,
            "link_body_list": body_names,
        }
        with open(args.save_path, "wb") as f:
            pickle.dump(motion_data, f)
        print(f"Saved to {args.save_path}")
            
      
    
    robot_motion_viewer.close()
