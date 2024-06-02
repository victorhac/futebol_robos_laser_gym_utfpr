from stable_baselines3 import PPO
from lib.environment.attacker.environment import Environment

model = PPO.load("models/attacker/PPO/2024_6_1_0_39_1/PPO_model")

env = Environment()

try:
    for i in range(100):
        next_state, _ = env.reset()
        reward = 0
        done = False

        while not done:
            action, state = model.predict(next_state)
            next_state, reward, done, _, _ = env.step(action)
            env.render()
finally:
    env.reset()