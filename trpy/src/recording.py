import json
import numpy as np
from matplotlib import pyplot as plt
import os 

dirName = "./recordings/"

class Recording:
    def __init__(self, data=None):
        self.data = data if data is not None else []  # List of {'timestamp', 'angles', 'gripper_state'}

    def save(self, filename):
        filename = os.path.join(dirName, filename)
        with open(filename, 'w') as f:
            json.dump(self.data, f)
        print(f"Recording saved to '{filename}'")

    @classmethod
    def load(cls, filename):
        if not os.path.exists(filename):
            filename = os.path.join(dirName, filename)
        if not os.path.exists(filename):
            raise FileExistsError("Invalid file path")
        with open(filename, 'r') as f:
            data = json.load(f)
        return cls(data)

    def plot(self):
        # Plot the joint angles over time
        if not self.data:
            print("No data to plot.")
            return

        timestamps = [dp['timestamp'] for dp in self.data]
        joint_ids = list(self.data[0]['angles'].keys())
        joint_ids = [int(jid) for jid in joint_ids]
        joint_angles = {jid: [] for jid in joint_ids}

        for dp in self.data:
            for jid in joint_ids:
                angle = dp['angles'][str(jid)]
                joint_angles[jid].append(angle)

        plt.figure()
        for jid in joint_ids:
            plt.plot(timestamps, joint_angles[jid], label=f'Joint {jid}')
        plt.xlabel('Time (s)')
        plt.ylabel('Angle (degrees)')
        plt.title('Joint Angles Over Time')
        plt.legend()
        plt.show()

    def slice(self, start_time, end_time):
        # Keep data within start_time and end_time
        self.data = [dp for dp in self.data if start_time <= dp['timestamp'] <= end_time]
