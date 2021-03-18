import gym
import os
import datetime
import time
from ruamel.yaml import YAML, dump, RoundTripDumper
from stable_baselines3 import PPO
from stable_baselines3.ppo import MlpPolicy
from stable_baselines3.common.callbacks import BaseCallback
from gyms.teacher.teacher import RaisimGymEnv
from raisimGymTorch.stable_baselines3.RaisimSbGymVecEnv import RaisimSbGymVecEnv as VecEnv
from raisimGymTorch.helper.raisim_gym_helper import ConfigurationSaver, load_param, tensorboard_launcher
import argparse

import wandb

#os.environ["WANDB_MODE"] = "dryrun"

# configuration
parser = argparse.ArgumentParser()
parser.add_argument('-w', '--weight', help='pre-trained weight path', type=str, default='')
args = parser.parse_args()
weight_path = args.weight

task_path = os.path.dirname(os.path.realpath(__file__))
rsc_path = task_path + "/../../rsc"
home_path = task_path +"/../.."

# 1. Start a new run
saver = ConfigurationSaver(log_dir=home_path + "/data/teacherFlat",
                           save_items=[task_path + "/cfg.yaml", task_path + "/Environment.hpp"])

# config
cfg = YAML().load(open(task_path + "/cfg.yaml", 'r'))

wandb.init(project='mmocrl_flat_teacher', entity='mgeisert', sync_tensorboard=True, config=cfg)

class SaverCallback(BaseCallback):
    def __init__(self, env, model, data_dir, freq=1000000, verbose=False):
        super(SaverCallback, self).__init__(verbose)
        self.env = env
        self.freq = freq
        self.data_dir = data_dir
        self.vis = False
        self.model = model

    def _on_step(self):
        rewards = env.wrapper.rewardInfo()
        rewardMean = rewards[0]

        for key in rewardMean:
            for i in range(1, len(rewards)):
                rewardMean[key] += rewards[i][key]
            rewardMean[key] = rewardMean[key] / float(len(rewards))

        rewardMean['reward_sum_ref'] = 0.05 * rewardMean['r_lv'] + 0.05 * rewardMean['r_av'] + 0.04 * rewardMean['r_b'] + 0.025 * rewardMean['r_s'] + 0.01 * rewardMean['r_fc'] + 0.00002 * rewardMean['r_t']

        wandb.log(rewardMean)
            
        if self.n_calls*env.num_envs % self.freq == 0:
            self.vis = True
            self.env.turn_on_visualization()
            self.env.start_video_recording(self.data_dir + "/" + datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S") + "policy_"+str(self.n_calls*self.env.num_envs)+'.mp4')
            model.save(self.data_dir + "/" + datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S") + "policy_"+str(self.n_calls*self.env.num_envs))
        
        if self.n_calls*env.num_envs % self.freq == env.num_envs * 800 and self.vis == True:
            self.vis = False
            self.env.turn_off_visualization()
            self.env.stop_video_recording()

        if self.vis == True:
            time.sleep(0.01)
        return True

# create environment from the configuration file
env = VecEnv(RaisimGymEnv(rsc_path, dump(cfg['environment'], Dumper=RoundTripDumper)), cfg['environment'])
env.turn_off_visualization()
obs = env.reset()

#wandb.config = cfg

n_steps = int(cfg['environment']['max_time'] / cfg['environment']['control_dt'])
policy_kwargs = dict(net_arch=[dict(pi=cfg['architecture']['policy_net'],vf=cfg['architecture']['value_net'])])
model = PPO(MlpPolicy, env,
            n_steps=n_steps,
            verbose=1,
            batch_size=int(n_steps*env.num_envs/cfg['training']['n_batch']),
            n_epochs=cfg['training']['n_epoch'], policy_kwargs=policy_kwargs, tensorboard_log=saver.data_dir)

if weight_path != '':
    print("Loading parameters from: ", weight_path)
    model = model.load(weight_path, env)

saver_callback = SaverCallback(env, model, saver.data_dir)
#wandb.watch(model)
update = 0


model.learn(total_timesteps=50000000, callback=saver_callback)

#  model.learn(total_timesteps=1000000)
#  env.turn_on_visualization()
#  #p = subprocess.Popen(["/home/mgeisert/raisim_ws/src/raisim/raisimUnity/linux/raisimUnity.x86_64"])
#  env.start_video_recording(datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S") + "policy_"+str(update)+'.mp4')
#
#  model.learn(total_timesteps=4000)
#
#  env.stop_video_recording()
#  env.turn_off_visualization()
#  #p.terminate()
#  model.learn(total_timesteps=1000000)
#  update = update + 1

