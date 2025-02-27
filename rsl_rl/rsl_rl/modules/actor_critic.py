#  Copyright 2021 ETH Zurich, NVIDIA CORPORATION
#  SPDX-License-Identifier: BSD-3-Clause


from __future__ import annotations

import torch
import torch.nn as nn
from torch.distributions import Normal


class ActorCritic(nn.Module):
    is_recurrent = False

    def __init__(
        self,
        num_obs,
        num_privileged_obs,
        num_obs_history,
        num_actions,
        actor_hidden_dims=[256, 256, 256],
        critic_hidden_dims=[256, 256, 256],
        adaptation_module_branch_hidden_dims = [[256, 32]],
        env_factor_encoder_branch_input_dims = [18],
        env_factor_encoder_branch_latent_dims = [18],
        env_factor_encoder_branch_hidden_dims = [[256, 128]],
        activation="elu",
        init_noise_std=1.0,
        **kwargs,
    ):
        if kwargs:
            print(
                "ActorCritic.__init__ got unexpected arguments, which will be ignored: "
                + str([key for key in kwargs.keys()])
            )
        super().__init__()

        activation = get_activation(activation)

        # mlp_input_dim_a = num_actor_obs
        # mlp_input_dim_c = num_critic_obs
        for i, (branch_input_dim, branch_hidden_dims, branch_latent_dim) in enumerate(
                zip(env_factor_encoder_branch_input_dims,
                    env_factor_encoder_branch_hidden_dims,
                    env_factor_encoder_branch_latent_dims)):
            # Env factor encoder
            env_factor_encoder_layers = []
            env_factor_encoder_layers.append(nn.Linear(branch_input_dim, branch_hidden_dims[0]))
            env_factor_encoder_layers.append(activation)
            for l in range(len(branch_hidden_dims)):
                if l == len(branch_hidden_dims) - 1:
                    env_factor_encoder_layers.append(
                        nn.Linear(branch_hidden_dims[l], branch_latent_dim))
                else:
                    env_factor_encoder_layers.append(
                        nn.Linear(branch_hidden_dims[l],
                                  branch_hidden_dims[l + 1]))
                    env_factor_encoder_layers.append(activation)
        self.env_factor_encoder = nn.Sequential(*env_factor_encoder_layers)
        self.add_module(f"encoder", self.env_factor_encoder)

        # Adaptation module
        for i, (branch_hidden_dims, branch_latent_dim) in enumerate(zip(adaptation_module_branch_hidden_dims,
                                                                        env_factor_encoder_branch_latent_dims)):
            adaptation_module_layers = []
            adaptation_module_layers.append(nn.Linear(num_obs_history, branch_hidden_dims[0]))
            adaptation_module_layers.append(activation)
            for l in range(len(branch_hidden_dims)):
                if l == len(branch_hidden_dims) - 1:
                    adaptation_module_layers.append(
                        nn.Linear(branch_hidden_dims[l], branch_latent_dim))
                else:
                    adaptation_module_layers.append(
                        nn.Linear(branch_hidden_dims[l],
                                  branch_hidden_dims[l + 1]))
                    adaptation_module_layers.append(activation)
        self.adaptation_module = nn.Sequential(*adaptation_module_layers)
        self.add_module(f"adaptation_module", self.adaptation_module)

        total_latent_dim = int(torch.sum(torch.Tensor(env_factor_encoder_branch_latent_dims)))

        # Policy
        actor_layers = []
        actor_layers.append(nn.Linear(total_latent_dim + num_obs, actor_hidden_dims[0]))
        actor_layers.append(activation)
        for l in range(len(actor_hidden_dims)):
            if l == len(actor_hidden_dims) - 1:
                actor_layers.append(nn.Linear(actor_hidden_dims[l], num_actions))
            else:
                actor_layers.append(nn.Linear(actor_hidden_dims[l], actor_hidden_dims[l + 1]))
                actor_layers.append(activation)
        self.actor_body = nn.Sequential(*actor_layers)

        # Value function
        critic_layers = []
        critic_layers.append(nn.Linear(total_latent_dim + num_obs, critic_hidden_dims[0]))
        critic_layers.append(activation)
        for l in range(len(critic_hidden_dims)):
            if l == len(critic_hidden_dims) - 1:
                critic_layers.append(nn.Linear(critic_hidden_dims[l], 1))
            else:
                critic_layers.append(nn.Linear(critic_hidden_dims[l], critic_hidden_dims[l + 1]))
                critic_layers.append(activation)
        self.critic_body = nn.Sequential(*critic_layers)

        print(f"Environment Factor Encoder: {self.env_factor_encoder}")
        print(f"Adaptation Module: {self.adaptation_module}")
        print(f"Actor MLP: {self.actor}")
        print(f"Critic MLP: {self.critic}")

        # Action noise
        self.std = nn.Parameter(init_noise_std * torch.ones(num_actions))
        self.distribution = None
        # disable args validation for speedup
        Normal.set_default_validate_args = False

        # seems that we get better performance without init
        # self.init_memory_weights(self.memory_a, 0.001, 0.)
        # self.init_memory_weights(self.memory_c, 0.001, 0.)

    @staticmethod
    # not used at the moment
    def init_weights(sequential, scales):
        [
            torch.nn.init.orthogonal_(module.weight, gain=scales[idx])
            for idx, module in enumerate(mod for mod in sequential if isinstance(mod, nn.Linear))
        ]

    def reset(self, dones=None):
        pass

    def forward(self):
        raise NotImplementedError

    @property
    def action_mean(self):
        return self.distribution.mean

    @property
    def action_std(self):
        return self.distribution.stddev

    @property
    def entropy(self):
        return self.distribution.entropy().sum(dim=-1)

    def update_distribution(self, observations, privileged_observations):
        latent = self.env_factor_encoder(privileged_observations)
        mean = self.actor_body(torch.cat((observations, latent), dim=-1))
        self.distribution = Normal(mean, mean * 0. + self.std)

    def act(self, observations, privileged_observations, **kwargs):
        self.update_distribution(observations, privileged_observations)
        return self.distribution.sample()

    def get_actions_log_prob(self, actions):
        return self.distribution.log_prob(actions).sum(dim=-1)

    def act_expert(self, ob, policy_info={}):
        return self.act_teacher(ob["obs"], ob["privileged_obs"])

    def act_inference(self, ob, policy_info={}):
        if ob["privileged_obs"] is not None:
            gt_latent = self.env_factor_encoder(ob["privileged_obs"])
            policy_info["gt_latents"] = gt_latent.detach().cpu().numpy()
        return self.act_student(ob["obs"], ob["obs_history"])

    def act_student(self, observations, observation_history, policy_info={}):
        latent = self.adaptation_module(observation_history)
        actions_mean = self.actor_body(torch.cat((observations, latent), dim=-1))
        policy_info["latents"] = latent.detach().cpu().numpy()
        return actions_mean

    def act_teacher(self, observations, privileged_info, policy_info={}):
        latent = self.env_factor_encoder(privileged_info)
        actions_mean = self.actor_body(torch.cat((observations, latent), dim=-1))
        policy_info["latents"] = latent.detach().cpu().numpy()
        return actions_mean

    def evaluate(self, critic_observations, privileged_observations, **kwargs):
        latent = self.env_factor_encoder(privileged_observations)
        value = self.critic_body(torch.cat((critic_observations, latent), dim=-1))
        return value


def get_activation(act_name):
    if act_name == "elu":
        return nn.ELU()
    elif act_name == "selu":
        return nn.SELU()
    elif act_name == "relu":
        return nn.ReLU()
    elif act_name == "crelu":
        return nn.CReLU()
    elif act_name == "lrelu":
        return nn.LeakyReLU()
    elif act_name == "tanh":
        return nn.Tanh()
    elif act_name == "sigmoid":
        return nn.Sigmoid()
    else:
        print("invalid activation function!")
        return None
