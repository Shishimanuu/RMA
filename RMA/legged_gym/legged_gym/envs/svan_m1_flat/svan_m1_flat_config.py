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
# this list of conditions and the following disclaimer in the documentationribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived FRom
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRE, INCIDENTA
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2021 ETH Zurich, Nikita Rudin

# SSH Connection established
# Test 1

from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO

class SvanM1FlatCfg( LeggedRobotCfg ):
    class init_state( LeggedRobotCfg.init_state ):
        pos = [0.0, 0.0, 0.6] # x,y,z [m]
        default_joint_angles = { # = target angles [rad] when action = 0.0
            'fr_abd_j': 0.1,   # [rad]
            'fl_abd_j': 0.1,   # [rad]
            'rr_abd_j': -0.1 ,  # [rad]
            'rl_abd_j': -0.1,   # [rad]

            'fr_hip_j':  0.8,   # [rad]
            'fl_hip_j':  1., # [rad]
            'rr_hip_j':  -0.8,   # [rad]
            'rl_hip_j':  1., # [rad]
                        
            'fr_knee_j': -1.5,  # [rad]
            'fl_knee_j': -1.5,   # [rad]
            'rr_knee_j': -1.5, # [rad]
            'rl_knee_j': -1.5,   # [rad]            
        }
    
    class terrain( LeggedRobotCfg.terrain ):
        mesh_type = 'plane'
        measure_heights = False

    class control( LeggedRobotCfg.control ):
        # PD Drive parameters:
        control_type = 'P'
        # stiffness = {'FR_hip_joint': 10, 'FR_thigh_joint': 5, 'FR_calf_joint': 2, 'FL_hip_joint': 10, 'FL_thigh_joint': 5, 'FL_calf_joint': 2,
        #             'RR_hip_joint': 10, 'RR_thigh_joint': 5, 'RR_calf_joint': 2, 'RL_hip_joint': 10, 'RL_thigh_joint': 5, 'RL_calf_joint': 2,
        # }  # [N*m/rad]
        # damping = {'FR_hip_joint': 8.94, 'FR_thigh_joint': 6.32, 'FR_calf_joint': 2.82, 'FL_hip_joint': 8.94, 'FL_thigh_joint': 6.32, 'FL_calf_joint': 2.82,
        #             'RR_hip_joint': 8.94, 'RR_thigh_joint': 6.32, 'RR_calf_joint': 2.82, 'RL_hip_joint': 8.94, 'RL_thigh_joint': 6.32, 'RL_calf_joint': 2.82,
        # }     # [N*m*s/rad]

        
        # stiffness = {"abd": 4, "hip": 2, "knee": 1}

        stiffness = {"j": 20}
        damping = {'j': 0.5} 
        # action scale: target angle = actionScale * action + defaultAngle
        action_scale = 0.2
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 4

    class asset( LeggedRobotCfg.asset ):
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/svan_m1/urdf/svan_urdf_package.urdf'
        name = "svan_m1"
        foot_name = "foot"
        fix_base_link = False
        penalize_contacts_on = ["thigh", "calf"]
        terminate_after_contacts_on = ["base"]
        self_collisions = 1 # 1 to disable, 0 to enable...bitwise filter
        flip_visual_attachments = False
  
    class rewards( LeggedRobotCfg.rewards ):
        soft_dof_pos_limit = 0.9
        base_height_target = 0.25
        feet_air_time =  2.0
        class scales( LeggedRobotCfg.rewards.scales ):
            torques = -0.0002
            dof_pos_limits = -10.0

class SvanM1FlatCfgPPO( LeggedRobotCfgPPO ):
    class algorithm( LeggedRobotCfgPPO.algorithm ):
        entropy_coef = 0.01
    class runner( LeggedRobotCfgPPO.runner ):
        run_name = ''
        experiment_name = 'Flat_svan_m1'

  