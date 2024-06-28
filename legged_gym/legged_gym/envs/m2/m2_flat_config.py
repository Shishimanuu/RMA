# SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived FRom
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2021 ETH Zurich, Nikita Rudin

from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO

class M2FlatCfg( LeggedRobotCfg ):
    class init_state( LeggedRobotCfg.init_state ):
        pos = [0.0, 0.0, 0.35] # x,y,z [m]


        default_joint_angles = { # = target angles [rad] when action = 0.0
            'FL_hip_joint': 0.1,   # [rad]
            'RL_hip_joint': 0.1,   # [rad]
            'FR_hip_joint': -0.1 ,  # [rad]
            'RR_hip_joint': -0.1,   # [rad]

            'FL_thigh_joint': 0.8,     # [rad]
            'RL_thigh_joint': 1.,   # [rad]
            'FR_thigh_joint': 0.8,     # [rad]
            'RR_thigh_joint': 1.,   # [rad]

            'FL_calf_joint': -1.5,   # [rad]
            'RL_calf_joint': -1.5,    # [rad]
            'FR_calf_joint': -1.5,  # [rad]
            'RR_calf_joint': -1.5,    # [rad]
        }

    class commands( LeggedRobotCfg.commands ):
        stance_int = 400
        stance_dur = 225
        stance_env_num_den = 192
        lin_vel_x = [-1.0, 1.0]
        lin_vel_y = [-1.0, 1.0]
        heading_command = False
        ang_vel_yaw = [-1.0, 1.0]
        resampling_time = 10.0
        body_height_cmd = [-0.25, 0.15]
        gait_frequency_cmd_range = [2.0, 4.0]
        gait_phase_cmd_range = [0.0, 1.0]
        gait_offset_cmd_range = [0.0, 1.0]
        gait_bound_cmd_range = [0.0, 1.0]
        gait_duration_cmd_range = [0.5, 0.5]
        footswing_height_range = [0.03, 0.09]
        body_pitch_range = [-0.4, 0.4]
        body_roll_range = [-0.0, 0.0]
        stance_width_range = [0.35, 0.45]
        stance_length_range = [0.35, 0.45]

        limit_vel_x = [-5.0, 5]
        limit_vel_y = [-0.6, 0.6]
        limit_vel_yaw = [-5.0, 5]
        limit_body_height = [-0.25, 0.]
        limit_gait_frequency = [2.0, 4.0]
        limit_gait_phase = [0.0, 1.0]
        limit_gait_offset = [0.0, 1.0]
        limit_gait_bound = [0.0, 1.0]
        limit_gait_duration = [0.5, 0.5]
        limit_footswing_height = [0.03, 0.09]
        limit_body_pitch = [-0.4, 0]
        limit_body_roll = [-0.0, 0]
        limit_stance_width = [0.35, 0.45]
        limit_stance_length = [0.35, 0.45]

        command_curriculum = True
        num_lin_vel_bins = 30
        num_ang_vel_bins = 30

        num_bins_vel_x = 21
        num_bins_vel_y = 1
        num_bins_vel_yaw = 21
        num_bins_body_height = 1
        num_bins_gait_frequency = 1
        num_bins_gait_phase = 1
        num_bins_gait_offset = 1
        num_bins_gait_bound = 1
        num_bins_gait_duration = 1
        num_bins_footswing_height = 1
        num_bins_body_roll = 1
        num_bins_body_pitch = 1
        num_bins_stance_width = 1

        exclusive_phase_offset = False
        pacing_offset = False
        binary_phases = True
        gaitwise_curricula = True
        
    class terrain( LeggedRobotCfg.terrain ):
        mesh_type = 'plane'
        # measure_heights = False
        # mesh_type = "trimesh"           # only changing this hampers training
        measure_heights = False             
        svan_terrain = False             # only changing this hampers training
        svan_curriculum = False          # only changing this hampers training
        svan_dyn_random = False
        curriculum = False
        terrain_length = 60
        restitution = -0.5
        # max_init_terrain_level = 0
        static_friction = 10
        dynamic_friction = 1
        max_terrain_level = 12
        visualize_force = False
        terrain_width = 60
        num_rows = 1 # number of terrain rows (levels)
        num_cols = 4 # number of terrain cols (types)

    class control( LeggedRobotCfg.control ):
        # PD Drive parameters:
        control_type = 'P'
        # stiffness = {'FR_hip_joint': 10, 'FR_thigh_joint': 5, 'FR_calf_joint': 2, 'FL_hip_joint': 10, 'FL_thigh_joint': 5, 'FL_calf_joint': 2,
        #             'RR_hip_joint': 10, 'RR_thigh_joint': 5, 'RR_calf_joint': 2, 'RL_hip_joint': 10, 'RL_thigh_joint': 5, 'RL_calf_joint': 2,
        # }  # [N*m/rad]
        # damping = {'FR_hip_joint': 8.94, 'FR_thigh_joint': 6.32, 'FR_calf_joint': 2.82, 'FL_hip_joint': 8.94, 'FL_thigh_joint': 6.32, 'FL_calf_joint': 2.82,
        #             'RR_hip_joint': 8.94, 'RR_thigh_joint': 6.32, 'RR_calf_joint': 2.82, 'RL_hip_joint': 8.94, 'RL_thigh_joint': 6.32, 'RL_calf_joint': 2.82,
        # }     # [N*m*s/rad]

        #60 and 0.5
        stiffness = { # = target angles [rad] when action = 0.0
            'FL_hip_joint':100,   # [rad]
            'RL_hip_joint':100,  # [rad]
            'FR_hip_joint':100,  # [rad]
            'RR_hip_joint':100,   # [rad]

            'FL_thigh_joint':60,     # [rad]
            'RL_thigh_joint':60,   # [rad]
            'FR_thigh_joint':60,     # [rad]
            'RR_thigh_joint':60,  # [rad]

            'FL_calf_joint':45,   # [rad]
            'RL_calf_joint':45,    # [rad]
            'FR_calf_joint':45,  # [rad]
            'RR_calf_joint':45,    # [rad]
        }

        damping = { # = target angles [rad] when action = 0.0
            'FL_hip_joint':0.5,   # [rad]
            'RL_hip_joint':0.5,  # [rad]
            'FR_hip_joint':0.5,  # [rad]
            'RR_hip_joint':0.5,   # [rad]

            'FL_thigh_joint':0.5,     # [rad]
            'RL_thigh_joint':0.5,   # [rad]
            'FR_thigh_joint':0.5,     # [rad]
            'RR_thigh_joint':0.5,  # [rad]

            'FL_calf_joint':0.5,   # [rad]
            'RL_calf_joint':0.5,    # [rad]
            'FR_calf_joint':0.5,  # [rad]
            'RR_calf_joint':0.5,    # [rad]
        }
        # stiffness = {'joint': 50}
        # damping = {'joint': 0.5} 
        # action scale: target angle = actionScale * action + defaultAngle
        action_scale = 0.20
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 4

    class asset( LeggedRobotCfg.asset ):
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/m2/urdf/SVANM2_URDF_inertia_change.urdf'
        # file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/a1/urdf/a1.urdf'
        mcp_running = False
        name = "m2"
        foot_name = "foot"
        fix_base_link = False
        penalize_contacts_on = ["thigh", "calf"]
        terminate_after_contacts_on = ["base", "hip", "calf"]
        # terminate_after_contacts_on = []
        self_collisions = 1 # 1 to disable, 0 to enable...bitwise filter
        flip_visual_attachments = False

    class domain_rand( LeggedRobotCfg.domain_rand ):
        push_robots = True
        randomize_base_mass = True
        randomize_friction = True
        added_mass_range = [-0.5, 0.5]
        lag_timesteps = 6
        randomize_lag_timesteps = True
        randomize_rigids_after_start = True
        randomize_friction_indep = True
        randomize_friction = True
        friction_range = [0.9, 3.0]
        randomize_restitution = True
        restitution_range = [0.0, 0.2]
        randomize_base_mass = True
        # added_mass_range = [-1.0, 3.0]
        randomize_gravity = False
        gravity_range = [0.95, 1.0]
        gravity_rand_interval_s = 8.0
        gravity_impulse_duration = 0.99
        randomize_com_displacement = True
        com_displacement_range = [-0.15, 0.15]
        randomize_ground_friction = True
        ground_friction_range = [15.0, 20.0]
        randomize_motor_strength = True
        motor_strength_range = [0.9, 1.1]
        randomize_motor_offset = True
        motor_offset_range = [-0.02, 0.02]
        push_robots = True
        randomize_Kp_factor = True
        randomize_Kd_factor = True

    class rewards( LeggedRobotCfg.rewards ):
        soft_dof_pos_limit = 0.9
        base_height_target = 0.3
        only_positive_rewards = True
        sigma_rew_neg = 0.02
        use_terminal_foot_height = False
        use_terminal_body_height = False
        terminal_body_height = 0.05
        use_terminal_roll_pitch = False
        terminal_body_ori = 1.6

        class scales( LeggedRobotCfg.rewards.scales ):
            torques = -0.0005
            dof_pos_limits = -10.0
            termination = -5.0
            tracking_lin_vel = 3.0
            tracking_ang_vel = 2.5
            # lin_vel_z = -0.05
            # ang_vel_xy = -0.05
            orientation = -0.05
            dof_acc = -2.5e-7
            base_height = -1.
            collision = -0.
            feet_stumble = -0.0 
            action_rate = -0.0002
            # stand_still = -0.5
            # stance_selective = -1.5
            feet_slip = -8e-4
            action_smoothness_1 = -2e-3
            action_smoothness_2 = -2e-3
            dof_vel = -2e-5
            dof_pos = -0.0
            jump = 10.0
            estimation_bonus = 0.0
            raibert_heuristic = -0.2
            feet_impact_vel = -0.0
            # feet_clearance = -1.0
            # feet_clearance_cmd = -1.0
            feet_clearance_cmd_linear = -11.0
            # orientation = 0.0
            orientation_control = -0.5*10
            tracking_stance_width = -0.0
            tracking_stance_length = -0.0
            lin_vel_z = -0.02
            ang_vel_xy = -0.001
            feet_air_time = 0.0
            hop_symmetry = 0.0
            tracking_contacts_shaped_force = 10.0
            tracking_contacts_shaped_vel = 10.0
            collision = -5.0

        penalty_level = {
            # 'action_rate_selective': 0,
            # 'orientation_selective': 0,
            # 'dof_vel_selective': 0,
            # 'dof_acc_selective': 0,
            # 'lin_vel_z_selective': 0, 
            # 'ang_vel_xy_selective': 0, 
        }

        stance_penalty = {
            # 'stance_selective': 0
            }

class M2FlatCfgPPO( LeggedRobotCfgPPO ):
    class algorithm( LeggedRobotCfgPPO.algorithm ):
        entropy_coef = 0.01
    class runner( LeggedRobotCfgPPO.runner ):
        run_name = ''
        experiment_name = 'Flat_m2'
        max_iterations = 2500


## Checking ssh cases
  