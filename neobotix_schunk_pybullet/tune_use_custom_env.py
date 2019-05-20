import ray
from ray import tune
import numpy as np
from env.neobotixschunkGymEnv import NeobotixSchunkGymEnv

def env_creator(env_config):
    env = NeobotixSchunkGymEnv(**env_config)
    return env

tune.register_env("my_env",env_creator)

ray.init()
tune.run(
    "PPO",
    num_samples= 1,
    checkpoint_freq=10,
    config={
        "env":"my_env",
    }
)