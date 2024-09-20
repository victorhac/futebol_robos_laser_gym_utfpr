from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv

from lib.domain.behavior_callback import BehaviorCallback
from lib.environment.attacker.environment import Environment
from lib.utils.behavior.behavior_utils import BehaviorUtils

import os
from datetime import datetime

task_training_name = "attacker"
algorithm_name = "PPO"

render_mode = "rgb_array"

num_threads = 12

total_timesteps = 200_000_000

gae_lambda = 0.95
gamma = 0.99
learning_rate = 0.0004
clip_range = 0.2
policy = "MlpPolicy"
batch_size = 128

device = "cpu"

load_model = True
loaded_model_path = "models/attacker/PPO/2024_9_19_23_56_39/PPO_model_task_1_update_100_3000000_steps"

updates_per_task = 100
check_count = 100
games_count = 200

log_interval = total_timesteps // 10

number_robot_blue = 3
number_robot_yellow = 3

def create_env(first_task_function):
    def _init():
        return Environment(first_task_function(), render_mode)
    return _init

def create_folder_if_not_exists(folder_path):
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)

def get_datetime_folder_name():
    current_datetime = datetime.now()

    year = current_datetime.year
    month = current_datetime.month
    day = current_datetime.day
    hour = current_datetime.hour
    minute = current_datetime.minute
    second = current_datetime.second

    return f"{year}_{month}_{day}_{hour}_{minute}_{second}"

def get_task_models_path():
    return f"models/{task_training_name}/{algorithm_name}/{get_datetime_folder_name()}"

def get_log_path():
    return f"log/{get_datetime_folder_name()}"

def main():
    get_task_1 = lambda: BehaviorUtils.get_task_1(updates_per_task)

    tasks = [
        get_task_1(),
        BehaviorUtils.get_task_2(updates_per_task),
        BehaviorUtils.get_task_3(updates_per_task),
        BehaviorUtils.get_task_4(updates_per_task),
        BehaviorUtils.get_task_5(updates_per_task),
        BehaviorUtils.get_task_6(updates_per_task),
        BehaviorUtils.get_task_7(updates_per_task)
    ]

    save_path = get_task_models_path()
    log_path = get_log_path()

    create_folder_if_not_exists(save_path)
    create_folder_if_not_exists(log_path)

    env = SubprocVecEnv([create_env(get_task_1) for _ in range(num_threads)])

    model = PPO(
        policy=policy,
        env=env,
        gamma=gamma,
        learning_rate=learning_rate,
        gae_lambda=gae_lambda,
        clip_range=clip_range,
        batch_size=batch_size,
        device=device)

    if load_model:
        model.set_parameters(loaded_model_path)

    checkpoint_callback = BehaviorCallback(
        check_count=check_count,
        total_timesteps=total_timesteps,
        model_name=algorithm_name,
        save_path=save_path,
        log_path=log_path,
        number_robot_blue=number_robot_blue,
        number_robot_yellow=number_robot_yellow,
        tasks=tasks,
        games_count=games_count)

    model.learn(
        total_timesteps=total_timesteps,
        log_interval=log_interval,
        callback=checkpoint_callback,
        progress_bar=True)
    
if __name__ == '__main__':
    main()