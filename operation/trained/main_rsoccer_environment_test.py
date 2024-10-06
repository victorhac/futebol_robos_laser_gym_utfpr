from stable_baselines3 import PPO
from lib.utils.behavior.behavior_utils import BehaviorUtils
from lib.environment.defensor.environment import Environment

task = BehaviorUtils.get_task_1(97)

env = Environment(task, "human")

model = PPO.load("models/attacker/PPO/2024_9_24_14_48_13/PPO_model_task_6_update_117_13999986_steps.zip")

env.opponent_model = model

try:
    for i in range(100):
        obs = env.reset()
        reward = 0
        done = False
        action = (0, 0)

        try:
            while done is False:
                next_state, reward, done, _, _ = env.step(action)
                env.render()

                action, _ = model.predict(next_state)
        except KeyboardInterrupt:
            env.reset()

        #input("Press Enter to continue...")
        env.reset()
        
except KeyboardInterrupt:
    env.reset()