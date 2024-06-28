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
mcp = False

obs_size = 48  # Adjust this based on your actual observation size
shm_name = 'obs_shm'  # Name of the shared memory block
if mcp:
    try:
        shm = shared_memory.SharedMemory(name=shm_name, create=True, size=obs_size * 8)  # 8 bytes per double
    except FileExistsError:
        shm = shared_memory.SharedMemory(name=shm_name)

def write_obs_to_shm(obs):
    obs_flat = np.ravel(obs.detach().cpu()).astype(np.float64)
    if mcp:
        np.ndarray((48, ), dtype=np.float64, buffer=shm.buf)[:obs_size] = obs_flat[:obs_size]
    else:
        np.ndarray(obs_flat.shape, dtype=np.float64, buffer=shm.buf)[:len(obs_flat)] = obs_flat[:obs_size]
    

def on_press(key):
    print("Key Pressed")
    update_command(key)

command = [0, 0, 0]
pause = False
write_data = 1

def update_command(key):
    global command
    global pause
    try:
        if key == keyboard.Key.up:
            command[0] += 0.2
        elif key == keyboard.Key.down:
            command[0] -= 0.2
        elif key == keyboard.Key.right:
            command[1] += 0.1
        elif key == keyboard.Key.left:
            command[1] -= 0.1
        elif key.char == 'y':
            command[2] += 0.1
        elif key.char == 't':
            command[2] -= 0.1
        elif key.char == 'z':
            pause = not pause
        elif key.char == 'q':
            write_data = 0
    except AttributeError:
        pass
    print("Command is: ", command)

def on_press(key):
    print("Key Pressed")
    update_command(key)

def play(args):
    
    env_cfg, train_cfg = task_registry.get_cfgs(name=args.task)
    global mcp
    mcp = args.mcp
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
    # load policy
    ppo_runner, train_cfg = task_registry.make_alg_runner(env=env, name=args.task, args=args, train_cfg=train_cfg)
    # policy = ppo_runner.get_inference_policy(device=env.device)

    policy = ppo_runner.get_inference_policy(device=env.device)
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
    global pause
    data_bytes = np.zeros(12).tobytes()
    if mcp:
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
        if mcp:
            actions = policy(obs[:, :obs_size].detach())    
        else:
            actions = policy(obs.detach())
        if mcp:
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

        tmp = actions[:, 0:3].clone()
        actions[:, 0:3] = actions[:, 3:6]
        actions[:, 3:6] = tmp
        actions[:, 0] *= -1
        actions[:, 3] *= -1
        obs, _, rews, dones, infos = env.step(actions.detach())
        write_obs_to_shm(obs)


        obs[0, 9] = command[0]
        obs[0, 10] = command[1]
        obs[0, 11] = command[2]
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
