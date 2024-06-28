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

class SvanM2FlatCfg( LeggedRobotCfg ):
    class init_state( LeggedRobotCfg.init_state ):
        pos = [0.0, 0.0, 0.6] # x,y,z [m]
        # default_joint_angles = { # = target angles [rad] when action = 0.0
        #     'FR_abd_j': 0.0,   # [rad]
        #     'FL_abd_j': 0.0,   # [rad]
        #     'RR_abd_j': -0.0 ,  # [rad]
        #     'RL_abd_j': -0.0,   # [rad]

        #     'FR_hip_j':  0.0,   # [rad]
        #     'FL_hip_j':  0., # [rad]
        #     'RR_hip_j':  0.0,   # [rad]
        #     'RL_hip_j':  0., # [rad]
                        
        #     'FR_knee_j':0.0,  # [rad]
        #     'FL_knee_j':0.0,   # [rad]
        #     'RR_knee_j':0.0, # [rad]
        #     'RL_knee_j':0.0,   # [rad]            
        # }

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
    
    class terrain( LeggedRobotCfg.terrain ):
        # mesh_type = 'plane'
        # measure_heights = False
        mesh_type = "trimesh"
        measure_heights = False
        svan_terrain = True
        svan_curriculum = False
        curriculum = False
        terrain_length = 16.
        terrain_width = 16.
        num_rows = 4 # number of terrain rows (levels)
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

        
        stiffness = {"joint": 30}
        damping = {'joint': 0.5} 
        # action scale: target angle = actionScale * action + defaultAngle
        action_scale = 0.25
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 4

    class asset( LeggedRobotCfg.asset ):
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/svan_m2/urdf/svan_urdf_package.urdf'
        name = "svan_m2"
        foot_name = "foot"
        fix_base_link = False
        penalize_contacts_on = ["thigh", "calf"]
        terminate_after_contacts_on = ["base"]
        self_collisions = 1 # 1 to disable, 0 to enable...bitwise filter
        flip_visual_attachments = False
  
    class domain_rand( LeggedRobotCfg.domain_rand ):
        push_robots = False

    class rewards( LeggedRobotCfg.rewards ):
        soft_dof_pos_limit = 0.9
        base_height_target = 0.25
        feet_air_time = 2.5
        class scales( LeggedRobotCfg.rewards.scales ):
            torques = -0.0002
            dof_pos_limits = -10.0

class SvanM2FlatCfgPPO( LeggedRobotCfgPPO ):
    class algorithm( LeggedRobotCfgPPO.algorithm ):
        entropy_coef = 0.01
    class runner( LeggedRobotCfgPPO.runner ):
        run_name = ''
        experiment_name = 'Flat_svan_m2'
  