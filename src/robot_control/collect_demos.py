import time
import threading
import os
import panda_py
from demo_recorder import KinestheticDemoRecorder

ROBOT_IP   = "172.16.0.2"
RECORD_HZ  = 20
DATA_DIR   = "../../data"
MIN_DEMOS  = 15

def recording_loop(recorder):
    """Runs in background — records at fixed Hz while is_recording=True."""
    while recorder.is_recording:
        recorder.record_step()
        time.sleep(recorder.dt)


def get_task_name():
    name = input("\nEnter task name (e.g. pick_up_cup): ").strip()
    if not name:
        name = "unnamed"
    return name


def confirm(prompt):
    """Ask yes/no question, return True if yes."""
    answer = input(f"{prompt} (y/n): ").strip().lower()
    return answer == "y"

def main():
    print("Franka Kinesthetic Demo Recorder")

    recorder = KinestheticDemoRecorder(
        robot_ip=ROBOT_IP,
        record_hz=RECORD_HZ
    )

    # Connection
    print(f"\nConnecting to Franka at {ROBOT_IP}")
    recorder.connect()
    print("Connected\n")


    # Task name
    task_name = get_task_name()
    save_path = os.path.join(DATA_DIR, task_name, "demos.hdf5")
    os.makedirs(os.path.dirname(save_path), exist_ok=True)
    print(f"Saving demos to: {save_path}")


    demo_count = 0

    while True:
        print(f"Demos recorded so far: {demo_count}\n")

        print("Options:\n\n")

        print("  [r]  Record new demo")
        print("  [s]  Save and quit" + (" (need more demos)" if demo_count < MIN_DEMOS else ""))
        print("  [q]  Quit without saving")

        choice = input("\nChoice: ").strip().lower()

        if choice == "r":

            # Move to home first for consistent start state
            print("\nMoving to home position")
            recorder.move_to_home()
            print("At home position.")

            input("Press Enter when ready to start recording")

            # Teaching mode
            recorder.enable_teaching_mode()
            print("Teaching mode ON — move the arm to demonstrate the task.")