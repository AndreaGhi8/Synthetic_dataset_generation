# Ghiotto Andrea   2118418

import sys
import os
import numpy as np

def generate_new_poses(input_file, output_file):
    with open(input_file, 'r') as infile, open(output_file, 'w') as outfile:
        for line in infile:
            pose_params = line.strip().split()
            
            # Original pose
            outfile.write(line)
            
            # -30°
            left30_yaw = float(pose_params[5]) + (30 * (np.pi / 180))
            left30_pose = f"{pose_params[0]} {pose_params[1]} {pose_params[2]} {pose_params[3]} {pose_params[4]} {left30_yaw}\n"
            outfile.write(left30_pose)

            # -15°
            left15_yaw = float(pose_params[5]) + (15 * (np.pi / 180))
            left15_pose = f"{pose_params[0]} {pose_params[1]} {pose_params[2]} {pose_params[3]} {pose_params[4]} {left15_yaw}\n"
            outfile.write(left15_pose)

            # +15°
            right15_yaw = float(pose_params[5]) - (15 * (np.pi / 180))
            right15_pose = f"{pose_params[0]} {pose_params[1]} {pose_params[2]} {pose_params[3]} {pose_params[4]} {right15_yaw}\n"
            outfile.write(right15_pose)

            # +30°
            right30_yaw = float(pose_params[5]) - (30 * (np.pi / 180))
            right30_pose = f"{pose_params[0]} {pose_params[1]} {pose_params[2]} {pose_params[3]} {pose_params[4]} {right30_yaw}\n"
            outfile.write(right30_pose)


if __name__ == "__main__":

    if len(sys.argv) != 2:
        print("Expected: python3 Generate_all_poses.py <folder>")
        sys.exit(1)
    
    folder = sys.argv[1]
    input_file = os.path.join(folder, "Grid_poses.txt")
    output_file = os.path.join(folder, "All_poses.txt")

    generate_new_poses(input_file, output_file)