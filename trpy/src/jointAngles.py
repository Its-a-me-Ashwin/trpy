class JointAngles:
    def __init__(self, angles):
        # Ensure that all keys in angles are integers
        self.angles = {int(k): v for k, v in angles.items()}  # Dictionary of joint angles {joint_id: angle}

    def __getitem__(self, key):
        key = int(key)  # Convert key to int
        try:
            return self.angles[key]
        except KeyError:
            raise KeyError(f"Joint ID {key} not found in angles.")

    def __setitem__(self, key, value):
        key = int(key)  # Convert key to int
        self.angles[key] = value

    def to_list(self, joint_ids):
        # Convert joint_ids to integers and retrieve corresponding angles
        return [self.angles[int(joint_id)] for joint_id in joint_ids]

    def __repr__(self):
        return f"{self.__class__.__name__}({self.angles})"
