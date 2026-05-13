"""
Test the custom ReachTask environment.
Run from src/simulation/:
    python test_reach_env.py
"""

import numpy as np
import robosuite as suite
from robosuite.controllers import load_composite_controller_config
from reach_env import ReachTask  # registers via @register_env

controller_config = load_composite_controller_config(robot="Panda")

env = suite.make(
    "ReachTask",
    robots="Panda",
    controller_configs=controller_config,
    has_renderer=True,
    render_camera="frontview",
    has_offscreen_renderer=False,
    use_object_obs=True,
    use_camera_obs=False,
    reward_shaping=True,
    control_freq=20,
)

obs = env.reset()
print("Obs keys:", list(obs.keys()))
print("target_pos:  ", obs.get("target_pos", "not found"))
print("eef_to_target:", obs.get("eef_to_target", "not found"))

low, high = env.action_spec

for step in range(300):
    action = np.random.uniform(low, high) * 0.05   # small random actions
    obs, reward, done, info = env.step(action)
    env.render()

    if step % 50 == 0:
        print(f"[{step}] reward={reward:.4f}  success={env._check_success()}"
              f"  dist={-reward:.4f}m")

    if done:
        print("Episode done, resetting")
        obs = env.reset()

env.close()
