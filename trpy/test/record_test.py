import asyncio
import numpy as np
import asyncio
import time
import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))
from robot import RobotArm, JointAngles
from robotConfigs import RobotData


async def main():
    # Create a RobotArm instance
    robot_name = "WX250"  # Replace with the actual robot name in your configuration
    robot = RobotArm(robot_name, port="COM3")  # Adjust port as needed

    try:
        # Record for 20 seconds
        print("Starting recording for 20 seconds...")
        recording_task = asyncio.create_task(robot.record())
        await asyncio.sleep(20)
        robot.stopRecording()

        # Wait for the recording to complete
        recording = await recording_task
        print(f"Recording completed. Recorded {len(recording.data)} data points.")

        # Play back the recorded motion
        print("Starting playback...")
        robot.playBack(recording)
        print("Playback completed.")

    except KeyboardInterrupt:
        print("Interrupted by user.")
        robot.stopRecording()

    finally:
        # Cleanup
        robot.close()

# Run the example
if __name__ == "__main__":
    asyncio.run(main())
