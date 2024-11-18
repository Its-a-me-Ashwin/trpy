import numpy as np
import asyncio
import time
import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))
from robot import RobotArm, JointAngles
from robotConfigs import RobotData

# Initialize the robot arm
robot = RobotArm("WX250", port="COM3")

# Define an asynchronous function to handle recording
async def record_motion(robot, duration, frequency):
    """
    Records the robot's motion for a specified duration and frequency.

    :param robot: The RobotArm instance.
    :param duration: Duration to record in seconds.
    :param frequency: Recording frequency in Hz.
    """
    # Start recording
    record_task = asyncio.create_task(robot.record(frequency=frequency))
    print("Recording started...")

    # Wait for the specified duration
    await asyncio.sleep(duration)

    # Stop recording
    robot.stopRecording()
    await record_task  # Ensure the recording task completes
    print("Recording stopped.")

# Function to perform the recording and playback
def main():
    try:
        # Example movement before recording
        print("Moving robot to initial position...")
        initial_angles = JointAngles({
            1: 0.0,
            2: 0.0,
            3: 0.0,
            4: -45.0,
            5: -40.0,
            6: 0.0,
            7: 0.0,
            8: 0.0,
            9: 0.0
        })
        robot.move(initial_angles, 0.2)

        input("Recording in 2 seconds, Press enter to start.")
        # Wait for robot to reach the position
        time.sleep(2)

        # Start recording asynchronously
        duration = 10  # Record for 10 seconds
        frequency = 10  # Record at 10 Hz
        asyncio.run(record_motion(robot, duration, frequency))

        # Playback the recorded motion
        input("Playing back the recorded motion. Press enter to start.")
        robot.playBack('recording.json')

        # Finish
        print("Playback completed.")

    finally:
        # Ensure the robot is properly closed
        robot.close()

# Run the main function
if __name__ == "__main__":
    print("Moving robot to initial position...")
    initial_angles = JointAngles({
        1: 25.0,
        2: 45.0,
        3: 45.0,
        4: -45.0,
        5: -45.0,
        6: 90.0,
        7: -30.0,
        8: -90.0,
        9: 0.0
    })
    robot.move(initial_angles, 0.2)
    robot.enable_torque(9)
    print("Open")
    robot.open_gripper()
    time.sleep(10)
    print("Close")
    robot.close_gripper()
    time.sleep(10)
    #main()
