# Ghiotto Andrea   2118418

import sys
import os
import numpy as np
from scipy.spatial.transform import Rotation as R

def pose_to_matrix(x, y, z, roll, pitch, yaw):
    rot = R.from_euler('xyz', [roll, pitch, yaw])
    T = np.eye(4)
    T[:3, :3] = rot.as_matrix()
    T[:3, 3] = [x, y, z]
    return T

def matrix_to_pose(T):
    rot = R.from_matrix(T[:3, :3])
    roll, pitch, yaw = rot.as_euler('xyz')
    x, y, z = T[:3, 3]
    return x, y, z, roll, pitch, yaw

def transform_poses(input_file, output_file):
    T = np.array([
        [1.0, 0.0, 0.130526496, 2.122],
        [0.0, 1.0, 0.0, 0.0012],
        [-0.130526496, 0.0, 0.991444821, -0.028474],
        [0.0, 0.0, 0.0, 1.0]
    ])

    with open(input_file, 'r') as infile, open(output_file, 'w') as outfile:
        for line in infile:
            values = list(map(float, line.strip().split()))

            x, y, z, roll, pitch, yaw = values
            T_pose = pose_to_matrix(x, y, z, roll, pitch, yaw)
            T_transformed = np.dot(T_pose, T)
            x_t, y_t, z_t, roll_t, pitch_t, yaw_t = matrix_to_pose(T_transformed)

            outfile.write(f"{x_t:.6f} {y_t:.6f} {z_t:.6f} {roll_t:.6f} {pitch_t:.6f} {yaw_t:.6f}\n")

if __name__ == "__main__":

    if len(sys.argv) != 2:
        print("Expected: python3 Transform_poses.py <folder>")
        sys.exit(1)
    
    folder = sys.argv[1]
    input_file = os.path.join(folder, "All_poses.txt")
    output_file = os.path.join(folder, "All_transformed_poses.txt")

    transform_poses(input_file, output_file)