from ray.tune.registry import register_env
from env.neobotixschunkGymEnv import NeobotixSchunkGymEnv
import ray
from ray.tune.logger import pretty_print
from ray.rllib.agents import ppo

def env_creator(env_config):
    env = NeobotixSchunkGymEnv(**env_config)
    return env

ray.init()
register_env("my_env",env_creator)
trainer = ppo.PPOTrainer(env="my_env")

for i in range(1000):
    result = trainer.train()
    print(pretty_print(result))
    if i % 10 == 0:
        checkpoint = trainer.save()
        print("checkpoint saved at", checkpoint)