from stable_baselines3 import PPO
from lib.environment.attacker.environment import Environment
import time

model = PPO.load("models/attacker/PPO/2024_6_12_9_49_55/PPO_model")

env = Environment("human")

episode_duration = 20

try:
    for i in range(100):
        next_state, _ = env.reset()
        reward = 0
        done = False
        episode_initial_time = time.time()

        while not done:
            action, state = model.predict(next_state)
            next_state, reward, done, _, _ = env.step(action)
            env.render()
finally:
    env.reset()