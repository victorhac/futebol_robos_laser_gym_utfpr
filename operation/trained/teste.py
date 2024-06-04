from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv
from stable_baselines3.common.callbacks import CheckpointCallback

from lib.environment.attacker.environment import Environment

import os
from datetime import datetime

task_training_name = "attacker"
algorithm_name = "PPO"

def create_env():
    def _init():
        return Environment()
    return _init

def create_folder_if_not_exists(folder_path):
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)

save_path = get_task_models_path()

create_folder_if_not_exists(save_path)

num_threads = 64
model_filename_prefix = f"{algorithm_name}_model"

env = SubprocVecEnv([create_env() for i in range(num_threads)])

n_actions = env.action_space.shape[-1]

gae_lambda = 0.95
gamma = 0.99
learning_rate = 0.0004
clip_range = 0.2
policy = "MlpPolicy"
batch_size = 128

load_model = True
loaded_model_path = "models/attacker/PPO/2024_6_3_16_46_10/PPO_model_90000000_steps"

model = PPO(
    policy=policy,
    env=env,
    gamma=gamma,
    gae_lambda=gae_lambda,
    clip_range=clip_range,
    batch_size=batch_size)

if load_model:
    model.set_parameters(loaded_model_path)

total_timesteps = 200_000_000

saved_model_number = 20
save_freq = total_timesteps // (saved_model_number * num_threads)
log_interval = total_timesteps // 10

checkpoint_callback = CheckpointCallback(
    save_freq=save_freq,
    save_path=save_path,
    name_prefix=model_filename_prefix)

model.learn(
    total_timesteps=total_timesteps,
    log_interval=log_interval,
    callback=checkpoint_callback,
    progress_bar=True)

model.save(f"{save_path}/{model_filename_prefix}")