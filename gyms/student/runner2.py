#!/usr/bin/env/python3
import gym
import os
import datetime
import time
from ruamel.yaml import YAML, dump, RoundTripDumper
from stable_baselines3 import PPO
from stable_baselines3.ppo import MlpPolicy
from stable_baselines3.common.callbacks import BaseCallback
from gyms.student.student import RaisimGymEnv
from gyms.student.CustomNetwork import *
from raisimGymTorch.stable_baselines3.RaisimSbGymVecEnv import RaisimSbGymVecEnv as VecEnv
from raisimGymTorch.helper.raisim_gym_helper import ConfigurationSaver, load_param, tensorboard_launcher

import argparse

import wandb

#Behaviro cloning
from imitation.algorithms.dagger import DAggerTrainer

os.environ["WANDB_MODE"] = "dryrun"

# configuration
parser = argparse.ArgumentParser()
parser.add_argument('-w', '--weight', help='pre-trained weight path', type=str, default='')
args = parser.parse_args()
weight_path = args.weight
#weight_path=''

task_path = os.path.dirname(os.path.realpath(__file__))
rsc_path = task_path + "/../../rsc"
home_path = task_path +"/../.."

# 1. Start a new run
saver = ConfigurationSaver(log_dir=home_path + "/data/student",
                           save_items=[task_path + "/cfg.yaml", task_path + "/Environment.hpp"])

# config
cfg = YAML().load(open(task_path + "/cfg.yaml", 'r'))

#wandb.init(project='mmocrl_flat_teacher', entity='mgeisert', sync_tensorboard=True, config=cfg)
wandb.init(project='mmocrl_student', entity='mgeisert', sync_tensorboard=True, config=cfg)
wandb.save(task_path + "/cfg.yaml")
wandb.save(task_path + "/Environment.hpp")

#make nested values
#if wandb.config['num_envs']:
#    print("num_envs set from wandb: ", wandb.config['num_envs'])
#    wandb.config['environment']['num_envs'] = wandb.config['num_envs']
#if wandb.config['num_threads']:
#    print("num_threads set from wandb: ", wandb.config['num_threads'])
#    wandb.config['environment']['num_threads'] = wandb.config['num_threads']
#if wandb.config['n_batch']:
#    print("n_batch set from wandb: ", wandb.config['n_batch'])
#    wandb.config['training']['n_batch'] = wandb.config['n_batch']
#if wandb.config['n_epoch']:
#    print("n_epoch set from wandb: ", wandb.config['n_epoch'])
#    wandb.config['training']['n_epoch'] = wandb.config['n_epoch']



class SaverCallback(BaseCallback):
    def __init__(self, env, freq=1000000, verbose=False):
        super(SaverCallback, self).__init__(verbose)
        self.env = env
        self.freq = freq
        self.vis = True
        self.env.turn_on_visualization()

    def _on_step(self):

        if self.n_calls % (400*4) == 0 and self.n_calls > 0:
            print("update terrains...")
            self.env.updateTerrains()

        rewards = env.wrapper.rewardInfo()
        rewardMean = rewards[0]

        for key in rewardMean:
            for i in range(1, len(rewards)):
                rewardMean[key] += rewards[i][key]
            rewardMean[key] = rewardMean[key] / float(len(rewards))

        rewardMean['reward_sum_ref'] = 0.05 * rewardMean['r_lv'] + 0.05 * rewardMean['r_av'] + 0.04 * rewardMean['r_b'] + 0.025 * rewardMean['r_s'] + 0.01 * rewardMean['r_fc'] + 0.00002 * rewardMean['r_t']

        wandb.log(rewardMean)
            
#        if self.n_calls*env.num_envs % self.freq == 0:
#            self.vis = True
#            self.env.turn_on_visualization()
#            #self.env.start_video_recording(self.data_dir + "/" + datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S") + "policy_"+str(self.n_calls*self.env.num_envs)+'.mp4')
#        
#        if self.n_calls*env.num_envs % self.freq == env.num_envs * 800 and self.vis == True:
#            self.vis = False
#            self.env.turn_off_visualization()
#            #self.env.stop_video_recording()

        #if self.vis == True:
        #    time.sleep(0.01)

        return True

# create environment from the configuration file

print("Create Enviornment")
env = VecEnv(RaisimGymEnv(rsc_path, dump(wandb.config['environment'], Dumper=RoundTripDumper)), wandb.config['environment'])
env.turn_on_visualization()
obs = env.reset()

#wandb.config = cfg

#n_steps = int(float(wandb.config['environment']['max_time']) / float(wandb.config['environment']['control_dt']))
#policy_kwargs = dict(net_arch=[dict(pi=wandb.config['architecture']['policy_net'],vf=wandb.config['architecture']['value_net'])],
#        features_extractor_class=PriviligedFeaturesExtractor, features_extractor_kwargs=dict())

print("load Policies")
student_policy = ExtendedStudentPolicy("/home/mgeisert/raisim_ws/src/mmocrl/data/teacherFlat/2021-07-06-11-41-11/2021-07-06-20-06-41policy_200000000.zip")
teacher_policy = ExtendedTeacherPolicy("/home/mgeisert/raisim_ws/src/mmocrl/data/teacherFlat/2021-07-06-11-41-11/2021-07-06-20-06-41policy_200000000.zip")
model = PPO.load("/home/mgeisert/raisim_ws/src/mmocrl/data/teacherFlat/2021-07-06-11-41-11/2021-07-06-20-06-41policy_200000000.zip")

print("Dagger")
dagger = DAggerTrainer(env, home_path + "/data/student/test", batch_size = 800)

dagger.bc_trainer.policy = student_policy

saver_callback = SaverCallback(env)

bc_args = dict()
bc_args['n_epochs'] = wandb.config['training']['n_epoch']
bc_args['n_batches'] = wandb.config['training']['n_batch']

print("Start training:")
for i in range(2000):
    collector = dagger.get_trajectory_collector()
    obs = collector.reset()
    for j in range(400):
        saver_callback._on_step()
        action = teacher_policy(obs)
        actionNumpy = action.detach().cpu().numpy()
        #print("actionNumpy shape: ", actionNumpy.shape)
        obs, _, _, _ = collector.step(actionNumpy)
        #action  = teacher_policy(obs)
        #print("action[0]: ", actionNumpy[0,:])

        #obs, _, _, _ = env.step(actionNumpy)
        print("substep: ", j)
    collector.saveTraj()
    print("step ", i)
    dagger.extend_and_update(bc_args)
    env.updateTerrain()
    if (i % 5):
        dagger.save_policy("/home/mgeisert/raisim_ws/src/mmocrl/data/teacherFlat/2021-07-06-11-41-11/student-"+i+".zip")


#wandb.watch(model)
update = 0

