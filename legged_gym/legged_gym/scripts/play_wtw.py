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
# contributors may be used to endorse or promote products derived from
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

from legged_gym import LEGGED_GYM_ROOT_DIR
import os
from multiprocessing import shared_memory
import isaacgym
from legged_gym.envs import *
from legged_gym.utils import  get_args, export_policy_as_jit, task_registry, Logger
from isaacgym import gymapi
from isaacgym import gymtorch

from pynput import keyboard

from pynput import keyboard

import numpy as np
import torch

obs_size = 70  # Adjust this based on your actual observation size
shm_name = 'obs_shm'  # Name of the shared memory block
try:
    shm = shared_memory.SharedMemory(name=shm_name, create=True, size=obs_size * 8)  # 8 bytes per double
except FileExistsError:
    shm = shared_memory.SharedMemory(name=shm_name)

def write_obs_to_shm(obs):
    obs_flat = np.ravel(obs.detach().cpu()).astype(np.float64)
    np.ndarray(obs_flat.shape, dtype=np.float64, buffer=shm.buf)[:len(obs_flat)] = obs_flat

backtick_pressed = False
commands = [0]*15
pause = False
gaits = {"pronking": [0, 0, 0],
            "trotting": [0.5, 0, 0],
            "bounding": [0, 0.5, 0],
            "pacing": [0, 0, 0.5]}

x_vel_cmd, y_vel_cmd, yaw_vel_cmd = 1.5, 0.0, 0.0
body_height_cmd = 0.0
step_frequency_cmd = 3.0
gait = torch.tensor(gaits["trotting"])
footswing_height_cmd = 0.08
pitch_cmd = 0.0
roll_cmd = 0.0
stance_width_cmd = 0.25

# initial commands
commands[0] = x_vel_cmd
commands[1] = y_vel_cmd
commands[2] = yaw_vel_cmd
commands[3] = body_height_cmd
commands[4] = step_frequency_cmd
commands[5:8] = gait
commands[8] = 0.5
commands[9] = footswing_height_cmd
commands[10] = pitch_cmd
commands[11] = roll_cmd
commands[12] = stance_width_cmdpause = False

write_data = 1

def update_command(key):
    global pause
    global backtick_pressed
    global gaits
    global commands
    gaits_global = gaits
    if key == keyboard.KeyCode(char='`'):
        backtick_pressed = True
        return  # Skip further processing for this key press

    if backtick_pressed:
        try:
            if key == keyboard.Key.up:
                commands[0] += 0.1
            elif key == keyboard.Key.down:
                commands[0] -= 0.1
            elif key == keyboard.Key.right:
                commands[1] += 0.1
            elif key == keyboard.Key.left:
                commands[1] -= 0.1
            elif key.char == 'd':
                commands[2] += 0.1
            elif key.char == 'a':
                commands[2] -= 0.1
            elif key.char == 'f':
                commands[4] += 0.5
            elif key.char == 'g':
                commands[4] -= 0.5
            elif key.char == '+':
                commands[3] += 0.1
            elif key.char == '-':
                commands[3] -= 0.1
            elif key.char == 'p':
                gait = gaits_global['pronking']
                gait_tensor = torch.tensor(gait) 
                print(f"gait tensor pronking: {gait_tensor}")
                commands[5:8] = gait_tensor
            elif key.char == 't':
                gait = gaits_global['trotting']
                gait_tensor = torch.tensor(gait) 
                commands[5:8] = gait_tensor
                print(f"gait tensor trotting: {gait_tensor}")
            elif key.char == 'b':
                gait = gaits_global['bounding']
                gait_tensor = torch.tensor(gait) 
                print(f"gait tensor bounding: {gait_tensor}")
                commands[5:8] = gait_tensor
            elif key.char == 'c':
                gait = gaits_global['pacing']
                gait_tensor = torch.tensor(gait) 
                print(f"gait tensor pacing: {gait_tensor}")
                commands[5:8] = gait_tensor
            elif key.char == '1':
                commands[9] += 0.1
            elif key.char == '2':
                commands[9] -= 0.1
            elif key.char == 'z':
                pause = not pause
        except AttributeError:
            print("Attribute Error")
            pass
        print("Command is: ", commands)

def on_press(key):
    print("Key Pressed")
    update_command(key)

def on_release(key):
    global backtick_pressed
    if key == keyboard.KeyCode(char='`'):
        backtick_pressed = False
        return 

def play(args):
    env_cfg, train_cfg = task_registry.get_cfgs(name=args.task)
    # override some parameters for testing
    env_cfg.env.num_envs = min(env_cfg.env.num_envs, 50)
    env_cfg.terrain.num_rows = 5
    env_cfg.terrain.num_cols = 5
    env_cfg.terrain.curriculum = False
    env_cfg.noise.add_noise = False
    env_cfg.domain_rand.randomize_friction = False
    env_cfg.domain_rand.push_robots = False

    # prepare environment
    env, _ = task_registry.make_env(name=args.task, args=args, env_cfg=env_cfg)
    obs = env.get_observations()



    # print("initial Observation 1: ", obs)
    # load policy
    train_cfg.runner.resume = True
    ppo_runner, train_cfg = task_registry.make_alg_runner(env=env, name=args.task, args=args, train_cfg=train_cfg)
    # policy = ppo_runner.get_inference_policy(device=env.device)
    # print("initial Observation 2: ", obs)

    policy = ppo_runner.get_inference_policy(device=env.device)
    # print("initial Observation 3: ", obs)
    # export policy as a jit module (used to run it from C++)
    if EXPORT_POLICY:
        path = os.path.join(LEGGED_GYM_ROOT_DIR, 'logs', train_cfg.runner.experiment_name, 'exported', 'policies')
        export_policy_as_jit(ppo_runner.alg.actor_critic, path)
        print('Exported policy as jit script to: ', path)
    logger = Logger(env.dt)
    robot_index = 0 # which robot is used for logging
    joint_index = 1 # which joint is used for logging
    stop_state_log = 100 # number of steps before plotting states
    stop_rew_log = env.max_episode_length + 1 # number of steps before print average episode rewards
    camera_position = np.array(env_cfg.viewer.pos, dtype=np.float64)
    camera_vel = np.array([1., 1., 0.])
    camera_direction = np.array(env_cfg.viewer.lookat) - np.array(env_cfg.viewer.pos)
    img_idx = 0
    plot = False
    global commands
    global pause
    data_bytes = np.zeros(12).tobytes()
    if env_cfg.asset.mcp_running:
        shm_name = 'joint_state'
        shm = shared_memory.SharedMemory( name = shm_name )

    obs_ind = [[0, 1, 2], [3, 4, 5], [9, 10, 11], [12, 13, 14, 15, 16, 17], [18, 19, 20, 21, 22, 23], [36, 37, 38, 39, 40, 41]]

    for i in range(10*int(env.max_episode_length)):

        if pause:
            while True:
                # print("On Break")
                if pause == False:
                    break
        default_joint_angles =  [0.1000,  0.8000, -1.5000, -0.1000,  0.8000, -1.5000,  0.1000,  1.0000, -1.5000, -0.1000,  1.0000, -1.5000]
        # default_joint_angles =  [0.4000,  0.4000, 0.40000, 0.40000,  0.40000, 0.40000,  0.40000,  0.40000, 0.4000, 0.40000,  0.40000, 0.4000]
        actions = policy(obs.detach())
        if env_cfg.asset.mcp_running:
            if i>0:
                data_bytes_2 = shm.buf[:len(data_bytes)]  # Adjust slice as needed
                data = np.frombuffer(data_bytes_2, dtype=np.float64)  # Adjust dtype as per your data
                actions = actions
                for idx in range(12):
                    if idx == 1 or idx == 2 or idx == 6 or idx == 9 or idx == 10 or idx == 11:
                        actions[0, idx] = 5 * (-float(data[idx]) - float(default_joint_angles[idx]))
                    else:    
                        # actions[0, idx] = float(data[idx]) - float(default_joint_angles[idx])   
                        actions[0, idx] = 5 * (float(data[idx]) - float(default_joint_angles[idx]))
            else:
                actions = 0*actions

        print("actions: ", actions)
        # actions = torch.tensor([0]*12)
        actions[0, :] = 0
        # actions[0, [0, 9]] = -0.4*5
        actions[0, [9]] = 0.4*5
        obs, _, rews, dones, infos = env.step(actions.detach())
        write_obs_to_shm(obs)
        print("Observations: ", obs)
        # print(f"Shape of Contact Forces: {env.contact_forces.shape}")
        # print(f"Contact Forces are: {env.contact_forces[0, [5, 9, 13, 17]]}")

        obs[0, 3:18] = torch.tensor(commands)
        action_init =  [0.1000,  0.8000, -1.5000, -0.1000,  0.8000, -1.5000,  0.1000,  1.0000, -1.5000, -0.1000,  1.0000, -1.5000]
        action_list = [float(act) for itr, act in enumerate(actions.detach()[0, :])]
        obs_list = [float(act) for act in obs.detach()[0, :]]

        if RECORD_FRAMES:
            if i % 2:
                filename = os.path.join(LEGGED_GYM_ROOT_DIR, 'logs', train_cfg.runner.experiment_name, 'exported', 'frames', f"{img_idx}.png")
                env.gym.write_viewer_image_to_file(env.viewer, filename)
                img_idx += 1 
        if MOVE_CAMERA:
            camera_position += camera_vel * env.dt
            env.set_camera(camera_position, camera_position + camera_direction)

        if  0 < i < stop_rew_log:
            if infos["episode"]:
                num_episodes = torch.sum(env.reset_buf).item()
                if num_episodes>0:
                    logger.log_rewards(infos["episode"], num_episodes)
        elif i==stop_rew_log:
            logger.print_rewards()
        

if __name__ == '__main__':
    EXPORT_POLICY = True
    RECORD_FRAMES = False
    MOVE_CAMERA = False
    listener = keyboard.Listener(on_press=on_press)
    listener.start()
    args = get_args()
    play(args)
    shm.close()
    shm.unlink() 
