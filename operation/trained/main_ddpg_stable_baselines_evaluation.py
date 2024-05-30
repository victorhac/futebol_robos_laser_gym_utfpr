from lib.environment.rsoccer_validation.environment import Environment
from lib.helpers.model_helper import ModelHelper

model = ModelHelper.get_defensor_model()

env = Environment()

try:
    for i in range(100):
        obs = env.reset()
        reward = 0
        done = False

        while done is False:
            action, state = model.predict(obs)
            obs, reward, done, info = env.step(action)
            env.render()
finally:
    env.reset()