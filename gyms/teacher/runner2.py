import gym
import os
import datetime

from ruamel.yaml import YAML, dump, RoundTripDumper
from stable_baselines3 import PPO
from stable_baselines3.ppo import MlpPolicy
from gyms.teacher.teacher import RaisimGymEnv
from raisimGymTorch.stable_baselines3.RaisimSbGymVecEnv import RaisimSbGymVecEnv as VecEnv


# Parallel environments
# directories
task_path = os.path.dirname(os.path.realpath(__file__))
rsc_path = task_path + "/../../rsc"

# config
cfg = YAML().load(open(task_path + "/cfg.yaml", 'r'))

# create environment from the configuration file
env = VecEnv(RaisimGymEnv(rsc_path, dump(cfg['environment'], Dumper=RoundTripDumper)), cfg['environment'])
obs = env.reset()
env.turn_off_visualization()

n_steps = int(cfg['environment']['max_time'] / cfg['environment']['control_dt'])
policy_kwargs = dict(net_arch=[128, 128])
model = PPO(MlpPolicy, env,
            n_steps=n_steps,
            verbose=1,
            batch_size=int(n_steps*env.num_envs/4),
            n_epochs=4, policy_kwargs=policy_kwargs)

update = 0
while(True):
  model.learn(total_timesteps=1000000)
  env.turn_on_visualization()
  #p = subprocess.Popen(["/home/mgeisert/raisim_ws/src/raisim/raisimUnity/linux/raisimUnity.x86_64"])
  env.start_video_recording(datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S") + "policy_"+str(update)+'.mp4')

  model.learn(total_timesteps=4000)

  env.stop_video_recording()
  env.turn_off_visualization()
  #p.terminate()
  model.learn(total_timesteps=1000000)
  update = update + 1

