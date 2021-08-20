from typing import Callable, Dict, List, Optional, Tuple, Type, Union

import gym
import torch as th
from torch import nn

from stable_baselines3.common.torch_layers import BaseFeaturesExtractor

#Priviledged information encoder and TCN can be set as "FeaturesExtractor" in the stable_baselines3 framework.

class PriviligedFeaturesExtractor(BaseFeaturesExtractor):
    def __init__(self, observation_space: gym.spaces.Space, state_dim: int = 133, priviliged_dim: int = 79, priviliged_arch:List[int] = [72, 64], activation_fn: Type[nn.Module]=nn.Tanh):
        super(PriviligedFeaturesExtractor, self).__init__(observation_space, state_dim + priviliged_arch[-1])

        self.priviliged_dim = priviliged_dim
        self.state_dim = state_dim

        priviliged_net = [] #computation of the latent vector for the priviliged information
        last_layer_dim_shared = priviliged_dim

        for layer_size in priviliged_arch:
            #print("layer_size: ", layer_size)
            priviliged_net.append(nn.Linear(last_layer_dim_shared, layer_size))
            priviliged_net.append(activation_fn())
            last_layer_dim_shared = layer_size

        self.priviliged_net = nn.Sequential(*priviliged_net)

    def forward(self, observation: th.Tensor) -> th.Tensor:
        state_obs, priviliged_obs = th.split(observation[:,:self.state_dim+self.priviliged_dim], [self.state_dim, self.priviliged_dim], 1)
        return th.cat([state_obs, self.priviliged_net(priviliged_obs)], 1)


#TODO:finsih History class by adding TCN
#class HistoryFeaturesExtractor(BaseFeaturesExtractor):    
#    def __init__(self, observation_space: gym.spaces.Space, state_dim: int = 133, history_dim: int = 79, priviliged_arch:List[int] = [72, 64], activation_fn: Type[nn.Module]=nn.Tanh):
#        super(HistoryFeaturesExtractor, self).__init__(observation_space, state_dim + priviliged_arch[-1])
#
#        self.history_dim = history_dim
#        self.state_dim = state_dim
#
#        history_net = [] #computation of the latent vector for the priviliged information
#        last_layer_dim_shared = priviliged_dim
#
#        for layer_size in priviliged_arch:
#            #print("layer_size: ", layer_size)
#            history_net.append(nn.Linear(last_layer_dim_shared, layer_size))
#            history_net.append(activation_fn())
#            last_layer_dim_shared = layer_size
#
#        self.history_net = nn.Sequential(*history_net)
#
#    def forward(self, observation: th.Tensor) -> th.Tensor:
#        state_obs, history_obs = th.split(observation, [self.state_dim, self.history_dim], 1)
#        return th.cat([state_obs, self.history_net(priviliged_obs)], 1)







        
