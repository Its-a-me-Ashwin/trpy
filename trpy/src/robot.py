import json
import numpy as np
import asyncio
import time
from dynamixel_sdk import *
import modern_robotics as mr
from robotConfigs import RobotData, pathToRobotData
from recording import Recording
import threading, keyboard
from jointAngles import JointAngles
import os

class RobotArm:
    def __init__(self, robot_name, port="COM4"):
        base_dir = os.path.dirname(os.path.abspath(__file__))
        robot_data_dir = os.path.join(base_dir, "robotData")

        config_file = os.path.join(robot_data_dir, f"{robot_name}.json")
        if not os.path.isfile(config_file):
            raise FileNotFoundError(f"Configuration file for {robot_name} not found in {robot_data_dir}.")
        
        M = RobotData[robot_name]["M"]
        Slist = RobotData[robot_name]["Slist"]

        print(config_file)
        # Load the configuration from the JSON file
        with open(config_file, 'r') as json_file:
            data = json.load(json_file)
            self.ID2NAME = {k: v for k, v in data.items() if k != "coupledJoints"}
            self.coupled_joints = {int(k): int(v) for k, v in data.get('coupledJoints', {}).items()}

        if M is None or Slist is None:
            raise ValueError("Missing matrices M or Slist.")
        self.M = M  # Home configuration matrix
        self.Slist = Slist  # Screw axis list
        self.joint_ids = [int(k) for k in self.ID2NAME.keys()]
        
        # Identify the gripper ID
        self.gripper_id = None
        for k, v in self.ID2NAME.items():
            if v.get("name", "").upper() == "GRIPPER":
                self.gripper_id = int(k)
                break

        # Create a list of joint IDs excluding the gripper
        self.motion_joint_ids = [joint_id for joint_id in self.joint_ids if joint_id != self.gripper_id]

        # Initialize PortHandler and PacketHandler instance
        self.portHandler = PortHandler(port)  # Update COM port as needed
        self.packetHandler = PacketHandler(2.0)

        # Open port
        if not self.portHandler.openPort():
            print("Failed to open the port")
            quit()
        else:
            print("Succeeded to open the port")

        # Set port baudrate
        if not self.portHandler.setBaudRate(1000000):
            print("Failed to change the baudrate")
            quit()
        else:
            print("Succeeded to change the baudrate")

        # Initialize other parameters
        self.ADDR_TORQUE_ENABLE = 64
        self.ADDR_GOAL_POSITION = 116
        self.ADDR_PRESENT_POSITION = 132
        self.ADDR_PROFILE_VELOCITY = 112
        self.TORQUE_ENABLE = 1
        self.TORQUE_DISABLE = 0

        # Initialize joint state
        self.current_angles = JointAngles({joint_id: 0.0 for joint_id in self.joint_ids})
        self.gripper_state = "OPEN"  # or "CLOSED"

    def angle_to_position(self, dxl_id, angle):
        info = self.ID2NAME[str(dxl_id)]
        min_pos = info["min"]
        max_pos = info["max"]
        if "minAngle" in info and "maxAngle" in info:
            min_angle = info["minAngle"]
            max_angle = info["maxAngle"]
            # Linear mapping from angle to position
            position = int(((angle - min_angle) * (max_pos - min_pos) / (max_angle - min_angle)) + min_pos)
        else:
            # For servos without minAngle and maxAngle (e.g., GRIPPER)
            # Assume angle is between 0 (open) and 1 (closed)
            position = int(angle * (max_pos - min_pos) + min_pos)
        return position

    def position_to_angle(self, dxl_id, position):
        info = self.ID2NAME[str(dxl_id)]
        min_pos = info["min"]
        max_pos = info["max"]
        if "minAngle" in info and "maxAngle" in info:
            min_angle = info["minAngle"]
            max_angle = info["maxAngle"]
            # Linear mapping from position to angle
            angle = ((position - min_pos) * (max_angle - min_angle) / (max_pos - min_pos)) + min_angle
        else:
            # For servos without minAngle and maxAngle (e.g., GRIPPER)
            # Return a value between 0 (open) and 1 (closed)
            angle = (position - min_pos) / (max_pos - min_pos)
        return angle

    def enable_torque(self, dxl_id):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, dxl_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE
        )
        if dxl_comm_result != COMM_SUCCESS:
            print(f"[ID:{dxl_id}] {self.packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            print(f"[ID:{dxl_id}] {self.packetHandler.getRxPacketError(dxl_error)}")

    def disable_torque(self, dxl_id):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, dxl_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE
        )
        if dxl_comm_result != COMM_SUCCESS:
            print(f"[ID:{dxl_id}] {self.packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            print(f"[ID:{dxl_id}] {self.packetHandler.getRxPacketError(dxl_error)}")

    def read_position(self, dxl_id):
        position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(
            self.portHandler, dxl_id, self.ADDR_PRESENT_POSITION
        )
        if dxl_comm_result != COMM_SUCCESS:
            print(f"[ID:{dxl_id}] {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            return None
        elif dxl_error != 0:
            print(f"[ID:{dxl_id}] {self.packetHandler.getRxPacketError(dxl_error)}")
            return None
        else:
            return position

    def FK(self, joint_angles):
        """
        Given joint angles, compute the end-effector position.
        """
        theta_list = joint_angles.to_list(self.joint_ids[:len(self.Slist[0])])
        T = mr.FKinSpace(self.M, self.Slist, theta_list)
        return T

    def IK(self, desired_T, initial_guess=None):
        """
        Given the desired end-effector pose, compute the joint angles.
        """
        if initial_guess is None:
            initial_guess = np.zeros(len(self.Slist[0]))
        else:
            initial_guess = initial_guess.to_list(self.joint_ids[:len(self.Slist[0])])

        eomg = 1e-3
        ev = 1e-3
        theta_list, success = mr.IKinSpace(self.Slist, self.M, desired_T, initial_guess, eomg, ev)
        if success:
            angles = {joint_id: theta for joint_id, theta in zip(self.joint_ids, theta_list)}
            return JointAngles(angles)
        else:
            print("IK did not converge")
            return None

    def set_profile_velocity(self, dxl_id, velocity):
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
            self.portHandler, dxl_id, self.ADDR_PROFILE_VELOCITY, velocity
        )
        if dxl_comm_result != COMM_SUCCESS:
            print(f"[ID:{dxl_id}] {self.packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            print(f"[ID:{dxl_id}] {self.packetHandler.getRxPacketError(dxl_error)}")
        else:
            print(f"[ID:{dxl_id}] Profile velocity set to {velocity}.")

    def set_profile_acceleration(self, dxl_id, acceleration):
        """
        Set the profile acceleration for a servo.
        """
        ADDR_PROFILE_ACCELERATION = 108  # Control table address for profile acceleration
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
            self.portHandler, dxl_id, ADDR_PROFILE_ACCELERATION, acceleration
        )
        if dxl_comm_result != COMM_SUCCESS:
            print(f"[ID:{dxl_id}] {self.packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            print(f"[ID:{dxl_id}] {self.packetHandler.getRxPacketError(dxl_error)}")

    def open_gripper(self):
        """
        Opens the gripper.
        """
        if self.gripper_id is None:
            raise IndexError("Gripper not equipped.")

        gripper_info = self.ID2NAME[str(self.gripper_id)]
        if "min" in gripper_info:
            target_position = gripper_info["min"]
        else:
            raise ValueError("GRIPPER servo does not have a 'min' position.")

        self.enable_torque(self.gripper_id)
        self.move_to_position(self.gripper_id, target_position)
        self.gripper_state = "OPEN"

    def close_gripper(self):
        """
        Closes the gripper.
        """
        if self.gripper_id is None:
            raise IndexError("Gripper not equipped.")

        gripper_info = self.ID2NAME[str(self.gripper_id)]
        if "max" in gripper_info:
            target_position = gripper_info["max"]
        else:
            raise ValueError("GRIPPER servo does not have a 'max' position.")

        self.enable_torque(self.gripper_id)
        self.move_to_position(self.gripper_id, target_position)
        self.gripper_state = "CLOSED"
    
    def move(self, joint_angles, duration):
        """
        Move the robot to the specified joint angles over the given duration.
        This is a blocking operation.

        :param joint_angles: JointAngles instance containing the target joint angles.
        :param duration: Movement duration in seconds.
        """
        # Ensure duration is positive
        duration = max(duration, 0.1)  # Minimum duration of 0.1 seconds to avoid division by zero

        # Enable torque on all servos
        for joint_id in self.joint_ids:
            self.enable_torque(joint_id)

        # Current and target positions
        current_positions = {joint_id: self.read_position(joint_id) for joint_id in self.joint_ids}
        target_positions = {joint_id: self.angle_to_position(joint_id, joint_angles[joint_id]) for joint_id in self.joint_ids}

        # Handle coupled joints in target positions
        for joint_id in self.coupled_joints:
            coupled_id = self.coupled_joints[joint_id]
            target_positions[coupled_id] = target_positions[joint_id]

        # Set profile velocity and acceleration
        for joint_id in self.joint_ids:
            distance = abs(target_positions[joint_id] - current_positions[joint_id])

            # Calculate required velocity and acceleration
            velocity = int(distance / duration)
            acceleration = int(velocity / duration)

            # Clamp velocity and acceleration to servo limits
            max_velocity = 1023  # Replace with actual max velocity for your servo
            max_acceleration = 32767  # Replace with actual max acceleration for your servo
            velocity = min(velocity, max_velocity)
            acceleration = min(acceleration, max_acceleration)

            self.set_profile_velocity(joint_id, velocity)
            self.set_profile_acceleration(joint_id, acceleration)

        # Command servos to move to target positions
        for joint_id in self.joint_ids:
            self.move_to_position(joint_id, target_positions[joint_id])

        # Wait for the movement to complete
        time.sleep(duration)

        # Update current_angles
        for joint_id in self.joint_ids:
            angle = joint_angles[joint_id]
            self.current_angles[joint_id] = angle  # Update current angle


    def release_servos(self):
        for joint_id in self.motion_joint_ids:
            self.disable_torque(joint_id)

    async def record(self, frequency=100):
        """
        Record the current joint angles at the specified frequency.
        This is an asynchronous operation.
        """
        self.recording = Recording()
        self.recording_active = True

        # Disable torque on all motion servos (excluding the gripper)
        for joint_id in self.motion_joint_ids:
            self.disable_torque(joint_id)

        interval = 1.0 / frequency

        start_time = time.time()

        # Start a separate thread to listen for keyboard inputs
        def gripper_control():
            while self.recording_active:
                if keyboard.is_pressed('c'):
                    #self.close_gripper()
                    self.gripper_state = "CLOSED"
                    print("Gripper Closed")
                    time.sleep(0.5)  # Debounce delay
                elif keyboard.is_pressed('o'):
                    #self.open_gripper()
                    self.gripper_state = "OPEN"
                    print("Gripper Opened")
                    time.sleep(0.5)  # Debounce delay
                time.sleep(0.01)

        gripper_thread = threading.Thread(target=gripper_control)
        gripper_thread.start()

        while self.recording_active:
            angles = {}
            for joint_id in self.motion_joint_ids:
                position = self.read_position(joint_id)
                if position is not None:
                    angle = self.position_to_angle(joint_id, position)
                    angles[joint_id] = angle
            timestamp = time.time() - start_time
            data_point = {'timestamp': timestamp, 'angles': angles, 'gripper_state': self.gripper_state}
            self.recording.data.append(data_point)
            await asyncio.sleep(interval)

        self.recording_active = False
        gripper_thread.join()
        print("Recording completed.")
        return self.recording

    def stopRecording(self):
        """
        Stop the recording process.
        """
        self.recording_active = False

    def playBack(self, recording, release_on_done=True):
        """
        Play back a recorded motion.
        This is a blocking operation.

        :param recording: Recording object containing the motion data.
        """
        # Ensure the recording is not empty
        if not recording.data:
            print("Recording is empty.")
            return

        # Enable torque on all motion servos (excluding the gripper)
        for joint_id in self.motion_joint_ids:
            self.enable_torque(joint_id)

        # Playback loop
        data_length = len(recording.data)
        for i in range(data_length):
            data_point = recording.data[i]
            angles = data_point['angles']
            gripper_state = data_point['gripper_state']

            # Handle coupled joints
            for joint_id in self.coupled_joints:
                coupled_id = self.coupled_joints[joint_id]
                if joint_id in angles:
                    angles[coupled_id] = angles[joint_id]

            # Command each servo
            for joint_id in self.motion_joint_ids:
                angle = angles[joint_id]
                position = self.angle_to_position(joint_id, angle)
                self.move_to_position(joint_id, position)

            # Control the gripper based on recorded state
            if gripper_state == "OPEN":
                self.open_gripper()
            elif gripper_state == "CLOSED":
                self.close_gripper()

            # Calculate sleep time to match the original timing
            if i < data_length - 1:
                current_time = recording.data[i]['timestamp']
                next_time = recording.data[i + 1]['timestamp']
                sleep_time = next_time - current_time
                time.sleep(sleep_time)

        if release_on_done:
            self.release_servos()


    def move_to_position(self, dxl_id, position):
        """
        Command the servo to move to a specified position.
        """
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
            self.portHandler, dxl_id, self.ADDR_GOAL_POSITION, int(position)
        )
        if dxl_comm_result != COMM_SUCCESS:
            print(f"[ID:{dxl_id}] {self.packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            print(f"[ID:{dxl_id}] {self.packetHandler.getRxPacketError(dxl_error)}")

    def close(self):
        """
        Close the port when done.
        """
        if self.portHandler is not None:
            self.portHandler.closePort()
            print("Port closed.")

    def __del__(self):
        try:
            self.release_servos()
        except AttributeError:  
            self.close()
        except Exception as e:
            print("Failed to stop the motors. Re initialize the class to stop.", e)
            self.close()
        finally:
            self.close()