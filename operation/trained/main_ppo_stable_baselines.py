from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv
from lib.domain.behavior_callback import BehaviorCallback

from lib.environment.attacker.environment import Environment

import os
from datetime import datetime

task_training_name = "attacker"
algorithm_name = "PPO"

def create_env():
    def _init():
        return Environment("human")
    return _init

def create_folder_if_not_exists(folder_path):
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)

def get_task_models_path():
    current_datetime = datetime.now()

    year = current_datetime.year
    month = current_datetime.month
    day = current_datetime.day
    hour = current_datetime.hour
    minute = current_datetime.minute
    second = current_datetime.second

    datetime_name = f"{year}_{month}_{day}_{hour}_{minute}_{second}"

    return f"models/{task_training_name}/{algorithm_name}/{datetime_name}"

def main():
    save_path = get_task_models_path()

    create_folder_if_not_exists(save_path)

    num_threads = 2

    env = SubprocVecEnv([create_env() for _ in range(num_threads)])

    gae_lambda = 0.95
    gamma = 0.99
    learning_rate = 0.0004
    clip_range = 0.2
    policy = "MlpPolicy"
    batch_size = 128

    load_model = False
    loaded_model_path = "models/attacker/PPO/2024_6_11_13_34_28/PPO_model"

    model = PPO(
        policy=policy,
        env=env,
        gamma=gamma,
        learning_rate=learning_rate,
        gae_lambda=gae_lambda,
        clip_range=clip_range,
        batch_size=batch_size)

    if load_model:
        model.set_parameters(loaded_model_path)

    total_timesteps = 200_000_000

    updates_per_task = 1000
    check_frequency = 5
    log_interval = total_timesteps // 10
    number_games = 100

    checkpoint_callback = BehaviorCallback(
        check_frequency=check_frequency,
        model_name=algorithm_name,
        save_path=save_path,
        number_robot_blue=3,
        number_robot_yellow=3,
        updates_per_task=updates_per_task,
        number_games=number_games)

    model.learn(
        total_timesteps=total_timesteps,
        log_interval=log_interval,
        callback=checkpoint_callback,
        progress_bar=True)
    
if __name__ == '__main__':
    main()