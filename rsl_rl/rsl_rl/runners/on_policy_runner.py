#  Copyright 2021 ETH Zurich, NVIDIA CORPORATION
#  SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import os
import copy
import statistics
import time
import torch
from collections import deque
from torch.utils.tensorboard import SummaryWriter as TensorboardSummaryWriter

import rsl_rl
from rsl_rl.algorithms import PPO
from rsl_rl.env import VecEnv
from rsl_rl.modules import ActorCritic, ActorCriticRecurrent, EmpiricalNormalization
from rsl_rl.utils import store_code_state

from legged_gym import LEGGED_GYM_ROOT_DIR

class DataCaches:
    def __init__(self, curriculum_bins):
        from rsl_rl.metrics_caches import DistCache, SlotCache

        self.slot_cache = SlotCache(curriculum_bins)
        self.dist_cache = DistCache()


caches = DataCaches(1)

class OnPolicyRunner:
    """On-policy runner for training and evaluation."""

    def __init__(self, env: VecEnv, train_cfg, log_dir=None, device="cpu"):
        self.cfg = train_cfg
        self.alg_cfg = train_cfg["algorithm"]
        self.policy_cfg = train_cfg["policy"]
        self.device = device
        self.env = env
        obs = self.env.get_observations()
        num_obs = obs.shape[1]

        # if "critic" in extras["observations"]:
        #     num_critic_obs = extras["observations"]["critic"].shape[1]
        # else:
        #     num_critic_obs = num_obs

        actor_critic_class = eval(self.policy_cfg.pop("class_name"))  # ActorCritic
        actor_critic: ActorCritic | ActorCriticRecurrent = actor_critic_class(
            num_obs, 
            self.env.num_privileged_obs,
            self.env.num_obs_history, 
            self.env.num_actions, 
            **self.policy_cfg
        ).to(self.device)
        alg_class = eval(self.alg_cfg.pop("class_name"))  # PPO
        self.alg: PPO = alg_class(actor_critic, device=self.device, **self.alg_cfg)
        self.num_steps_per_env = self.cfg["num_steps_per_env"]
        self.save_interval = self.cfg["save_interval"]
        # self.empirical_normalization = self.cfg["empirical_normalization"]
        
        # if self.empirical_normalization:
        #     self.obs_normalizer = EmpiricalNormalization(shape=[num_obs], until=1.0e8).to(self.device)
        #     # self.critic_obs_normalizer = EmpiricalNormalization(shape=[num_critic_obs], until=1.0e8).to(self.device)
        # else:
        #     self.obs_normalizer = torch.nn.Identity()  # no normalization
        #     # self.critic_obs_normalizer = torch.nn.Identity()  # no normalization
        # init storage and model
        self.alg.init_storage(
            self.env.num_train_envs, 
            self.num_steps_per_env, 
            [self.env.num_obs],
            [self.env.num_privileged_obs],
            [self.env.num_obs_history],
            [self.env.num_actions])

        # Log
        self.log_dir = log_dir
        self.writer = None
        self.tot_timesteps = 0
        self.tot_time = 0
        self.current_learning_iteration = 0
        self.git_status_repos = [rsl_rl.__file__]

    def learn(self, num_learning_iterations, init_at_random_ep_len=False, eval_freq=100, eval_expert=False):
        from ml_logger import logger
        # initialize writer
        if self.log_dir is not None and self.writer is None:
            # Launch either Tensorboard or Neptune & Tensorboard summary writer(s), default: Tensorboard.
            self.logger_type = self.cfg.get("logger", "tensorboard")
            self.logger_type = self.logger_type.lower()

            if self.logger_type == "neptune":
                from rsl_rl.utils.neptune_utils import NeptuneSummaryWriter

                self.writer = NeptuneSummaryWriter(log_dir=self.log_dir, flush_secs=10, cfg=self.cfg)
                self.writer.log_config(self.env.cfg, self.cfg, self.alg_cfg, self.policy_cfg)
            elif self.logger_type == "wandb":
                from rsl_rl.utils.wandb_utils import WandbSummaryWriter

                self.writer = WandbSummaryWriter(log_dir=self.log_dir, flush_secs=10, cfg=self.cfg)
                self.writer.log_config(self.env.cfg, self.cfg, self.alg_cfg, self.policy_cfg)
            elif self.logger_type == "tensorboard":
                self.writer = TensorboardSummaryWriter(log_dir=self.log_dir, flush_secs=10)
            else:
                raise AssertionError("logger type not found")

        if init_at_random_ep_len:
            self.env.episode_length_buf = torch.randint_like(
                self.env.episode_length_buf, high=int(self.env.max_episode_length)
            )

        num_train_envs = self.env.num_train_envs

        obs_dict = self.env.get_observations()
        # critic_obs = extras["observations"].get("critic", obs)
        # obs, critic_obs = obs.to(self.device), critic_obs.to(self.device)
        obs, privileged_obs, obs_history = obs_dict["obs"], obs_dict["privileged_obs"], obs_dict["obs_history"]
        obs, privileged_obs, obs_history = obs.to(self.device), privileged_obs.to(self.device), obs_history.to(
            self.device)
        self.train_mode()  # switch to train mode (for dropout for example)

        ep_infos = []
        rewbuffer = deque(maxlen=100)
        lenbuffer = deque(maxlen=100)
        rewbuffer_eval = deque(maxlen=100)
        lenbuffer_eval = deque(maxlen=100)
        cur_reward_sum = torch.zeros(self.env.num_envs, dtype=torch.float, device=self.device)
        cur_episode_length = torch.zeros(self.env.num_envs, dtype=torch.float, device=self.device)

        if hasattr(self.env, "curriculum"):
            caches.__init__(curriculum_bins=len(self.env.curriculum))

        start_iter = self.current_learning_iteration
        tot_iter = start_iter + num_learning_iterations
        for it in range(start_iter, tot_iter):
            start = time.time()
            # Rollout
            with torch.inference_mode():
                for i in range(self.num_steps_per_env):
                    actions_train = self.alg.act(obs[:num_train_envs], privileged_obs[:num_train_envs],
                                                 obs_history[:num_train_envs])
                    if eval_expert:
                        actions_eval = self.alg.actor_critic.act_teacher(obs[num_train_envs:],
                                                                         privileged_obs[num_train_envs:])
                    else:
                        actions_eval = self.alg.actor_critic.act_student(obs[num_train_envs:],
                                                                         privileged_obs[num_train_envs:])
                    ret = self.env.step(torch.cat((actions_train, actions_eval), dim=0))
                    obs_dict, rewards, dones, infos = ret
                    obs, privileged_obs, obs_history = obs_dict["obs"], obs_dict["privileged_obs"], obs_dict[
                        "obs_history"]

                    # if "critic" in infos["observations"]:
                    #     critic_obs = self.critic_obs_normalizer(infos["observations"]["critic"])
                    # else:
                    #     critic_obs = obs
                    # obs, critic_obs, rewards, dones = (
                    #     obs.to(self.device),
                    #     critic_obs.to(self.device),
                    #     rewards.to(self.device),
                    #     dones.to(self.device),
                    # )
                    obs, privileged_obs, obs_history, rewards, dones = obs.to(self.device), privileged_obs.to(
                        self.device), obs_history.to(self.device), rewards.to(self.device), dones.to(self.device)
                    self.alg.process_env_step(rewards[:num_train_envs], dones[:num_train_envs], infos)


                #     if self.log_dir is not None:
                #         # Book keeping
                #         # note: we changed logging to use "log" instead of "episode" to avoid confusion with
                #         # different types of logging data (rewards, curriculum, etc.)
                #         if "episode" in infos:
                #             ep_infos.append(infos["episode"])
                #         elif "log" in infos:
                #             ep_infos.append(infos["log"])
                #         cur_reward_sum += rewards
                #         cur_episode_length += 1
                #         new_ids = (dones > 0).nonzero(as_tuple=False)
                #         rewbuffer.extend(cur_reward_sum[new_ids][:, 0].cpu().numpy().tolist())
                #         lenbuffer.extend(cur_episode_length[new_ids][:, 0].cpu().numpy().tolist())
                #         cur_reward_sum[new_ids] = 0
                #         cur_episode_length[new_ids] = 0

                # stop = time.time()
                # collection_time = stop - start

                # # Learning step
                # start = stop
                if 'train/episode' in infos:
                        with logger.Prefix(metrics="train/episode"):
                            logger.store_metrics(**infos['train/episode'])

                if 'eval/episode' in infos:
                    with logger.Prefix(metrics="eval/episode"):
                        logger.store_metrics(**infos['eval/episode'])

                if 'curriculum' in infos:
                    curr_bins_train = infos['curriculum']['reset_train_env_bins']
                    curr_bins_eval = infos['curriculum']['reset_eval_env_bins']

                    caches.slot_cache.log(curr_bins_train, **{
                        k.split("/", 1)[-1]: v for k, v in infos['curriculum'].items()
                        if k.startswith('slot/train')
                        })
                    caches.slot_cache.log(curr_bins_eval, **{
                        k.split("/", 1)[-1]: v for k, v in infos['curriculum'].items()
                        if k.startswith('slot/eval')
                    })
                    caches.dist_cache.log(**{
                        k.split("/", 1)[-1]: v for k, v in infos['curriculum'].items()
                        if k.startswith('dist/train')
                    })
                    caches.dist_cache.log(**{
                        k.split("/", 1)[-1]: v for k, v in infos['curriculum'].items()
                        if k.startswith('dist/eval')
                    })

                    cur_reward_sum += rewards
                    cur_episode_length += 1

                    new_ids = (dones > 0).nonzero(as_tuple=False)

                    new_ids_train = new_ids[new_ids < num_train_envs]
                    rewbuffer.extend(cur_reward_sum[new_ids_train].cpu().numpy().tolist())
                    lenbuffer.extend(cur_episode_length[new_ids_train].cpu().numpy().tolist())
                    cur_reward_sum[new_ids_train] = 0
                    cur_episode_length[new_ids_train] = 0

                    new_ids_eval = new_ids[new_ids >= num_train_envs]
                    rewbuffer_eval.extend(cur_reward_sum[new_ids_eval].cpu().numpy().tolist())
                    lenbuffer_eval.extend(cur_episode_length[new_ids_eval].cpu().numpy().tolist())
                    cur_reward_sum[new_ids_eval] = 0
                    cur_episode_length[new_ids_eval] = 0

               # Learning step
                self.alg.compute_returns(obs[:num_train_envs], privileged_obs[:num_train_envs])

                if it % eval_freq == 0:
                    self.env.reset_evaluation_envs()

                if it % eval_freq == 0:
                    logger.save_pkl({"iteration": it,
                                     **caches.slot_cache.get_summary(),
                                     **caches.dist_cache.get_summary()},
                                    path=f"curriculum/info.pkl", append=True)

            mean_value_loss, mean_surrogate_loss, mean_adaptation_module_loss = self.alg.update()

            # mean_value_loss, mean_surrogate_loss = self.alg.update()
        #     stop = time.time()
        #     learn_time = stop - start
        #     self.current_learning_iteration = it
        #     if self.log_dir is not None:
        #         self.log(locals())
        #     if it % self.save_interval == 0:
        #         self.save(os.path.join(self.log_dir, f"model_{it}.pt"))
        #     ep_infos.clear()
        #     if it == start_iter:
        #         # obtain all the diff files
        #         git_file_paths = store_code_state(self.log_dir, self.git_status_repos)
        #         # if possible store them to wandb
        #         if self.logger_type in ["wandb", "neptune"] and git_file_paths:
        #             for path in git_file_paths:
        #                 self.writer.save_file(path)

        # self.save(os.path.join(self.log_dir, f"model_{self.current_learning_iteration}.pt"))
            logger.store_metrics(
                time_elapsed=logger.since('start'),
                time_iter=logger.split('epoch'),
                adaptation_loss=mean_adaptation_module_loss,
                mean_value_loss=mean_value_loss,
                mean_surrogate_loss=mean_surrogate_loss
            )

            # if self.save_video_interval:
            #     self.log_video(it)

            self.tot_timesteps += self.num_steps_per_env * self.env.num_envs
            if logger.every(self.log_freq, "iteration", start_on=1):
                # if it % Config.log_freq == 0:
                logger.log_metrics_summary(key_values={"timesteps": self.tot_timesteps, "iterations": it})
                logger.job_running()

            if it % self.save_interval == 0:
                with logger.Sync():
                    logger.torch_save(self.alg.actor_critic.state_dict(), f"checkpoints/ac_weights_{it:06d}.pt")
                    logger.duplicate(f"checkpoints/ac_weights_{it:06d}.pt", f"checkpoints/ac_weights_last.pt")

                    path = f'{LEGGED_GYM_ROOT_DIR}/tmp/legged_data'

                    os.makedirs(path, exist_ok=True)

                    adaptation_module_path = f'{path}/adaptation_module_latest.jit'
                    adaptation_module = copy.deepcopy(self.alg.actor_critic.adaptation_module).to('cpu')
                    traced_script_adaptation_module = torch.jit.script(adaptation_module)
                    traced_script_adaptation_module.save(adaptation_module_path)

                    body_path = f'{path}/body_latest.jit'
                    body_model = copy.deepcopy(self.alg.actor_critic.actor_body).to('cpu')
                    traced_script_body_module = torch.jit.script(body_model)
                    traced_script_body_module.save(body_path)

                    logger.upload_file(file_path=adaptation_module_path, target_path=f"checkpoints/", once=False)
                    logger.upload_file(file_path=body_path, target_path=f"checkpoints/", once=False)

            self.current_learning_iteration += num_learning_iterations

        with logger.Sync():
            logger.torch_save(self.alg.actor_critic.state_dict(), f"checkpoints/ac_weights_{it:06d}.pt")
            logger.duplicate(f"checkpoints/ac_weights_{it:06d}.pt", f"checkpoints/ac_weights_last.pt")

            path = f'{LEGGED_GYM_ROOT_DIR}/tmp/legged_data'

            os.makedirs(path, exist_ok=True)

            adaptation_module_path = f'{path}/adaptation_module_latest.jit'
            adaptation_module = copy.deepcopy(self.alg.actor_critic.adaptation_module).to('cpu')
            traced_script_adaptation_module = torch.jit.script(adaptation_module)
            traced_script_adaptation_module.save(adaptation_module_path)

            body_path = f'{path}/body_latest.jit'
            body_model = copy.deepcopy(self.alg.actor_critic.actor_body).to('cpu')
            traced_script_body_module = torch.jit.script(body_model)
            traced_script_body_module.save(body_path)

            logger.upload_file(file_path=adaptation_module_path, target_path=f"checkpoints/", once=False)
            logger.upload_file(file_path=body_path, target_path=f"checkpoints/", once=False)

    # def log(self, locs: dict, width: int = 80, pad: int = 35):
    #     self.tot_timesteps += self.num_steps_per_env * self.env.num_envs
    #     self.tot_time += locs["collection_time"] + locs["learn_time"]
    #     iteration_time = locs["collection_time"] + locs["learn_time"]

    #     ep_string = ""
    #     if locs["ep_infos"]:
    #         for key in locs["ep_infos"][0]:
    #             infotensor = torch.tensor([], device=self.device)
    #             for ep_info in locs["ep_infos"]:
    #                 # handle scalar and zero dimensional tensor infos
    #                 if key not in ep_info:
    #                     continue
    #                 if not isinstance(ep_info[key], torch.Tensor):
    #                     ep_info[key] = torch.Tensor([ep_info[key]])
    #                 if len(ep_info[key].shape) == 0:
    #                     ep_info[key] = ep_info[key].unsqueeze(0)
    #                 infotensor = torch.cat((infotensor, ep_info[key].to(self.device)))
    #             value = torch.mean(infotensor)
    #             # log to logger and terminal
    #             if "/" in key:
    #                 self.writer.add_scalar(key, value, locs["it"])
    #                 ep_string += f"""{f'{key}:':>{pad}} {value:.4f}\n"""
    #             else:
    #                 self.writer.add_scalar("Episode/" + key, value, locs["it"])
    #                 ep_string += f"""{f'Mean episode {key}:':>{pad}} {value:.4f}\n"""
    #     mean_std = self.alg.actor_critic.std.mean()
    #     fps = int(self.num_steps_per_env * self.env.num_envs / (locs["collection_time"] + locs["learn_time"]))

    #     self.writer.add_scalar("Loss/value_function", locs["mean_value_loss"], locs["it"])
    #     self.writer.add_scalar("Loss/surrogate", locs["mean_surrogate_loss"], locs["it"])
    #     self.writer.add_scalar("Loss/learning_rate", self.alg.learning_rate, locs["it"])
    #     self.writer.add_scalar("Policy/mean_noise_std", mean_std.item(), locs["it"])
    #     self.writer.add_scalar("Perf/total_fps", fps, locs["it"])
    #     self.writer.add_scalar("Perf/collection time", locs["collection_time"], locs["it"])
    #     self.writer.add_scalar("Perf/learning_time", locs["learn_time"], locs["it"])
    #     if len(locs["rewbuffer"]) > 0:
    #         self.writer.add_scalar("Train/mean_reward", statistics.mean(locs["rewbuffer"]), locs["it"])
    #         self.writer.add_scalar("Train/mean_episode_length", statistics.mean(locs["lenbuffer"]), locs["it"])
    #         if self.logger_type != "wandb":  # wandb does not support non-integer x-axis logging
    #             self.writer.add_scalar("Train/mean_reward/time", statistics.mean(locs["rewbuffer"]), self.tot_time)
    #             self.writer.add_scalar(
    #                 "Train/mean_episode_length/time", statistics.mean(locs["lenbuffer"]), self.tot_time
    #             )

    #     str = f" \033[1m Learning iteration {locs['it']}/{locs['tot_iter']} \033[0m "

    #     if len(locs["rewbuffer"]) > 0:
    #         log_string = (
    #             f"""{'#' * width}\n"""
    #             f"""{str.center(width, ' ')}\n\n"""
    #             f"""{'Computation:':>{pad}} {fps:.0f} steps/s (collection: {locs[
    #                         'collection_time']:.3f}s, learning {locs['learn_time']:.3f}s)\n"""
    #             f"""{'Value function loss:':>{pad}} {locs['mean_value_loss']:.4f}\n"""
    #             f"""{'Surrogate loss:':>{pad}} {locs['mean_surrogate_loss']:.4f}\n"""
    #             f"""{'Mean action noise std:':>{pad}} {mean_std.item():.2f}\n"""
    #             f"""{'Mean reward:':>{pad}} {statistics.mean(locs['rewbuffer']):.2f}\n"""
    #             f"""{'Mean episode length:':>{pad}} {statistics.mean(locs['lenbuffer']):.2f}\n"""
    #         )
    #         #   f"""{'Mean reward/step:':>{pad}} {locs['mean_reward']:.2f}\n"""
    #         #   f"""{'Mean episode length/episode:':>{pad}} {locs['mean_trajectory_length']:.2f}\n""")
    #     else:
    #         log_string = (
    #             f"""{'#' * width}\n"""
    #             f"""{str.center(width, ' ')}\n\n"""
    #             f"""{'Computation:':>{pad}} {fps:.0f} steps/s (collection: {locs[
    #                         'collection_time']:.3f}s, learning {locs['learn_time']:.3f}s)\n"""
    #             f"""{'Value function loss:':>{pad}} {locs['mean_value_loss']:.4f}\n"""
    #             f"""{'Surrogate loss:':>{pad}} {locs['mean_surrogate_loss']:.4f}\n"""
    #             f"""{'Mean action noise std:':>{pad}} {mean_std.item():.2f}\n"""
    #         )
    #         #   f"""{'Mean reward/step:':>{pad}} {locs['mean_reward']:.2f}\n"""
    #         #   f"""{'Mean episode length/episode:':>{pad}} {locs['mean_trajectory_length']:.2f}\n""")

    #     log_string += ep_string
    #     log_string += (
    #         f"""{'-' * width}\n"""
    #         f"""{'Total timesteps:':>{pad}} {self.tot_timesteps}\n"""
    #         f"""{'Iteration time:':>{pad}} {iteration_time:.2f}s\n"""
    #         f"""{'Total time:':>{pad}} {self.tot_time:.2f}s\n"""
    #         f"""{'ETA:':>{pad}} {self.tot_time / (locs['it'] + 1) * (
    #                            locs['num_learning_iterations'] - locs['it']):.1f}s\n"""
    #     )
    #     print(log_string)

    # def save(self, path, infos=None):
    #     saved_dict = {
    #         "model_state_dict": self.alg.actor_critic.state_dict(),
    #         "optimizer_state_dict": self.alg.optimizer.state_dict(),
    #         "iter": self.current_learning_iteration,
    #         "infos": infos,
    #     }
    #     if self.empirical_normalization:
    #         saved_dict["obs_norm_state_dict"] = self.obs_normalizer.state_dict()
    #         saved_dict["critic_obs_norm_state_dict"] = self.critic_obs_normalizer.state_dict()
    #     torch.save(saved_dict, path)

    #     # Upload model to external logging service
    #     if self.logger_type in ["neptune", "wandb"]:
    #         self.writer.save_model(path, self.current_learning_iteration)

    # def load(self, path, load_optimizer=True):
    #     loaded_dict = torch.load(path)
    #     self.alg.actor_critic.load_state_dict(loaded_dict["model_state_dict"])
    #     if self.empirical_normalization:
    #         self.obs_normalizer.load_state_dict(loaded_dict["obs_norm_state_dict"])
    #         self.critic_obs_normalizer.load_state_dict(loaded_dict["critic_obs_norm_state_dict"])
    #     if load_optimizer:
    #         self.alg.optimizer.load_state_dict(loaded_dict["optimizer_state_dict"])
    #     self.current_learning_iteration = loaded_dict["iter"]
    #     return loaded_dict["infos"]

    def get_inference_policy(self, device=None):
        self.alg.actor_critic.eval()
        if device is not None:
            self.alg.actor_critic.to(device)
        return self.alg.actor_critic.act_inference

    def get_expert_policy(self, device=None):
        self.alg.actor_critic.eval()
        if device is not None:
            self.alg.actor_critic.to(device)
        return self.alg.actor_critic.act_expert

    # def get_inference_policy(self, device=None):
    #     self.eval_mode()  # switch to evaluation mode (dropout for example)
    #     if device is not None:
    #         self.alg.actor_critic.to(device)
    #     policy = self.alg.actor_critic.act_inference
    #     if self.cfg["empirical_normalization"]:
    #         if device is not None:
    #             self.obs_normalizer.to(device)
    #         policy = lambda x: self.alg.actor_critic.act_inference(self.obs_normalizer(x))  # noqa: E731
    #     return policy

    # def train_mode(self):
    #     self.alg.actor_critic.train()
    #     if self.empirical_normalization:
    #         self.obs_normalizer.train()
    #         self.critic_obs_normalizer.train()

    # def eval_mode(self):
    #     self.alg.actor_critic.eval()
    #     if self.empirical_normalization:
    #         self.obs_normalizer.eval()
    #         self.critic_obs_normalizer.eval()

    # def add_git_repo_to_log(self, repo_file_path):
    #     self.git_status_repos.append(repo_file_path)
