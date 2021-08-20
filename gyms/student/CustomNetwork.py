from typing import Callable, Dict, List, Optional, Tuple, Type, Union

import gym
import torch as th
from torch import nn
import numpy as np

from stable_baselines3.common.policies import BasePolicy
from stable_baselines3 import PPO

#Extended networks output [action, latent_estimation] instead  of just action so Behavior Cloning learns to copy both.

class ExtendedStudentPolicy(BasePolicy):
    def __init__(self, path_to_model, state_dim: int = 133, privileged_dim: int = 79, history_dim: int = 6000, fully_connected_arch:List[int] = [64], activation_fn: Type[nn.Module]=nn.Tanh):
        super(ExtendedStudentPolicy, self).__init__(state_dim+privileged_dim+history_dim, 16 + 64)

        self.state_dim = state_dim
        self.privileged_dim = privileged_dim
        self.history_dim = history_dim

        TCN_net = [] #computation of the latent vector for the privileged information
        last_layer_dim_shared = privileged_dim

        self.channel = 60

        #Environment Estimation
        TCN_net.append(nn.Conv1d(self.channel, self.channel, 5, stride=1, dilation=1, groups=self.channel))
        TCN_net.append(nn.ReLU())
        TCN_net.append(nn.Conv1d(self.channel, self.channel, 5, stride=2, dilation=1, groups=self.channel))
        TCN_net.append(nn.ReLU())
        TCN_net.append(nn.Conv1d(self.channel, self.channel, 5, stride=1, dilation=2, groups=self.channel))
        TCN_net.append(nn.ReLU())
        TCN_net.append(nn.Conv1d(self.channel, self.channel, 5, stride=2, dilation=1, groups=self.channel))
        TCN_net.append(nn.ReLU())
        TCN_net.append(nn.Conv1d(self.channel, self.channel, 5, stride=1, dilation=4, groups=self.channel))
        TCN_net.append(nn.ReLU())
        #TCN_net.append(nn.Conv1d(self.channel, self.channel, 5, stride=2, dilation=1, groups=self.channel))
        #TCN_net.append(nn.ReLU)

        last_layer_dim_shared = self.channel
        #fully_connected_net = []
        TCN_net.append(nn.Flatten())

        for layer_size in fully_connected_arch:
            #print("layer_size: ", layer_size)
            TCN_net.append(nn.Linear(last_layer_dim_shared, layer_size))
            TCN_net.append(activation_fn())
            last_layer_dim_shared = layer_size

        #self.fully_connected_net = nn.Sequential(*fully_connected_net)
        self.TCN_net = nn.Sequential(*TCN_net)

        #Load policy from from teacher
        model = PPO.load(path_to_model)
        self.action_net = nn.Sequential(*[model.policy.mlp_extractor.policy_net, model.policy.action_net])

    def forward(self, observation: th.Tensor) -> th.Tensor:
        state_obs, privileged_obs, history_obs = th.split(observation, [self.state_dim, self.privileged_dim, self.history_dim], 1)
        history_obs.reshape([observation.size()[0], self.channel, self.history_dim/self.channel])
        latent_estimation = self.TCN_net(history_obs)
        return th.cat([self.action_net(th.cat([obs[:, self.state_dim], latent_estimation], 1)), latent_estimation], 1)

    def _predict(self, obs):
        return forward(obs)


class ExtendedTeacherPolicy:
    def __init__(self, path_to_model, state_dim: int = 133, privileged_dim: int = 79, history_dim: int = 6000):
        self.model = PPO.load(path_to_model)
        self.state_dim = state_dim
        self.privileged_dim = privileged_dim
        self.history_dim = history_dim
    def __call__(self, obs):
        with th.no_grad():
            thobs = th.Tensor(obs).cuda()
            print("b", thobs.requires_grad)

            #print("size:", thobs.size())
            #print("size out feature_extractor: ", (self.model.policy.features_extractor(thobs[:, :self.state_dim+self.privileged_dim])[:,self.state_dim:]).size())
            features = self.model.policy.features_extractor(thobs[:,:self.state_dim+self.privileged_dim])
            #print("features[0]: ", features[0,:])
            mlp_actions = self.model.policy.mlp_extractor(features)[0]
            #print("mlp_actions[0]: ", mlp_actions[0,:])
            actions = self.model.policy.action_net(mlp_actions)
            #print("actions[0]: ", actions[0,:])
            return th.cat([actions, features[:,self.state_dim:]], 1)


        
