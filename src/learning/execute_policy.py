import torch
import numpy as np
import panda_py
import panda_py.controllers
import time
import sys
import os
from scipy.spatial.transform import Rotation
import robomimic.utils.file_utils as FileUtils
import robomimic.utils.torch_utils as TorchUtils

ROBOT_IP     = "172.16.0.2"
POLICY_PATH  = "/home/borys/Desktop/bachalor_thesis_lfd/models/wave/wave_bc/20260417152510/models/model_epoch_200.pth"
HORIZON      = 200
HZ           = 20
DT           = 1.0 / HZ
ACTION_SCALE = 0.15  # Scaling for safety


def get_obs(state):
    """Convert robot state to policy observation."""
    q        = np.array(state.q)
    eef_pos  = np.array(state.O_T_EE[12:15])
    T        = np.array(state.O_T_EE).reshape(4, 4, order='F')
    eef_quat = Rotation.from_matrix(T[:3, :3]).as_quat()  # [x,y,z,w]

    return {
        "robot0_joint_pos":    torch.tensor(q,          dtype=torch.float32).unsqueeze(0),
        "robot0_eef_pos":      torch.tensor(eef_pos,    dtype=torch.float32).unsqueeze(0),
        "robot0_eef_quat":     torch.tensor(eef_quat,   dtype=torch.float32).unsqueeze(0),
        "robot0_gripper_qpos": torch.tensor([0.0, 0.0], dtype=torch.float32).unsqueeze(0),
    }


def main():
    # Load policy
    print("Loading policy.")
    device = TorchUtils.get_torch_device(try_to_use_cuda=False)
    policy, _ = FileUtils.policy_from_checkpoint(
        ckpt_path=POLICY_PATH,
        device=device,
        verbose=True
    )
    policy.start_episode()
    print("Policy loaded")

    # Connect
    print(f"\nConnecting to Franka at {ROBOT_IP}")
    panda = panda_py.Panda(ROBOT_IP)
    print("Connected")

    # Go home
    print("Moving home.")
    panda.move_to_start()

    input("\nPress Enter to start execution.")

    # Running policy
    print(f"Running policy for {HORIZON} steps at {HZ}Hz")
    print("Press Ctrl+C to stop")

    # Cartesian impedance controller
    controller = panda_py.controllers.CartesianImpedance()

    try:
        with panda.create_context(frequency=HZ, max_runtime=HORIZON / HZ) as ctx:
            while ctx.ok():
                # Get state
                state = panda.get_state()
                obs   = get_obs(state)

                # Get action from policy
                with torch.no_grad():
                    action = policy(obs)
                action = np.array(action).flatten()  # (7,)

                delta_pos = action[:3] * ACTION_SCALE
                delta_ori = action[3:6] * ACTION_SCALE
                # action[6] = gripper to be added

                # Current eef pose
                current_pos  = np.array(state.O_T_EE[12:15])
                T            = np.array(state.O_T_EE).reshape(4, 4, order='F')
                current_rot  = Rotation.from_matrix(T[:3, :3])

                # Target eef pose
                target_pos = current_pos + delta_pos
                target_rot = Rotation.from_euler('xyz', delta_ori) * current_rot

                # Build 4x4 target transform
                target_mat = np.eye(4)
                target_mat[:3, :3] = target_rot.as_matrix()
                target_mat[:3,  3] = target_pos

                # Send to Cartesian controller
                controller.set_control(
                    panda,
                    position=target_pos,
                    orientation=target_rot.as_quat()
                )

    except KeyboardInterrupt:
        print("Stopped")

    finally:
        print("\nGoing home.")
        panda.move_to_start()
        print("Done")


if __name__ == "__main__":
    main()