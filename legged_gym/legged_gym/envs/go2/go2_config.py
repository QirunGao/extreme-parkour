# SPDX-FileCopyrightText: Copyright (c) 2021
# SPDX-License-Identifier: BSD-3-Clause

from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO

class Go2RoughCfg(LeggedRobotCfg):
    class init_state(LeggedRobotCfg.init_state):
        # Initial pose (can be tuned)
        pos = [0.0, 0.0, 0.42]  # x,y,z [m]
        default_joint_angles = {  # = target angles [rad] when action = 0.0
            # Hips
            'FL_hip_joint':  0.10,   # [rad]
            'RL_hip_joint':  0.10,   # [rad]
            'FR_hip_joint': -0.10,   # [rad]
            'RR_hip_joint': -0.10,   # [rad]
            # Thighs
            'FL_thigh_joint': 1.00,  # [rad]
            'RL_thigh_joint': 1.00,  # [rad]
            'FR_thigh_joint': 1.00,  # [rad]
            'RR_thigh_joint': 1.00,  # [rad]
            # Calves
            'FL_calf_joint': -1.50,  # [rad]
            'RL_calf_joint': -1.50,  # [rad]
            'FR_calf_joint': -1.50,  # [rad]
            'RR_calf_joint': -1.50,  # [rad]
        }

    class init_state_slope(LeggedRobotCfg.init_state):
        # Alternative start on slope/ramps (optional)
        pos = [0.56, 0.0, 0.24]  # x,y,z [m]
        default_joint_angles = {  # = target angles [rad] when action = 0.0
            'FL_hip_joint':  0.03,   # [rad]
            'RL_hip_joint':  0.03,   # [rad]
            'FR_hip_joint': -0.03,   # [rad]
            'RR_hip_joint': -0.03,   # [rad]

            'FL_thigh_joint': 1.00,  # [rad]
            'RL_thigh_joint': 1.90,  # [rad]
            'FR_thigh_joint': 1.00,  # [rad]
            'RR_thigh_joint': 1.90,  # [rad]

            'FL_calf_joint': -2.20,  # [rad]
            'RL_calf_joint': -0.90,  # [rad]
            'FR_calf_joint': -2.20,  # [rad]
            'RR_calf_joint': -0.90,  # [rad]
        }

    class control(LeggedRobotCfg.control):
        # PD Drive parameters:
        control_type = 'P'
        stiffness = {'joint': 30.0}   # [N*m/rad]
        damping   = {'joint': 0.6}    # [N*m*s/rad]
        # action scale: target angle = action_scale * action + defaultAngle
        action_scale = 0.25
        # decimation: number of control action updates @ sim DT per policy DT
        decimation  = 4

    class asset(LeggedRobotCfg.asset):
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/go2/urdf/go2.urdf'
        foot_name = "foot"
        penalize_contacts_on = ["thigh", "calf"]
        terminate_after_contacts_on = ["base"]  # add more links if needed
        self_collisions = 1  # 1 to disable, 0 to enable... bitwise filter

    class depth(LeggedRobotCfg.depth):
        use_camera = True
        # Sensor's native output resolution (for rendering)
        original = (848, 480)         # (W, H)
        # Downsampled resolution fed to the network (matches repo/paper)
        resized  = (58, 87)           # (W, H)
        buffer_len = 4                # Frame buffer length (temporal)
        update_interval = 1           # Render every N env steps
        near_clip = 0.10
        far_clip  = 5
        dis_noise = 0.01              # Simple distance noise (hardware emulation)
        horizontal_fov = 87.0         # Horizontal field of view (deg)

        # Camera mount pose relative to the base (front face, slight downward tilt)
        position = [0.32715, -0.00003, 0.04297]  # Offset in base frame [m]
        angle    = [-5.0, -5.0]       # Pitch angle range (deg); sampled at attach time


class Go2RoughCfgPPO(LeggedRobotCfgPPO):
    class algorithm(LeggedRobotCfgPPO.algorithm):
        entropy_coef = 0.01
    class runner(LeggedRobotCfgPPO.runner):
        run_name = ''
        experiment_name = 'rough_go2'