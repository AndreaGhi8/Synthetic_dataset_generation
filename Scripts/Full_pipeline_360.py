# Ghiotto Andrea   2118418

import subprocess
import sys
import time
import os
import numpy as np

index = 0 # Global variable to name images progressively

def get_pose(object_name):
    # Run the script Get_pose.py
    print("Acquiring the current pose of the robot...")
    result = subprocess.run(['python3', '/tmp/Scripts/Get_pose.py',
                             '-n', object_name],
                             capture_output=True, text=True)
    if result.returncode != 0:
        print("Error recovering pose:", result.stderr)
        sys.exit(1)
    return result.stdout.strip()
    
def get_sonar_image(output_folder, index):
    # Run the script Get_sonar_images.py
    print("Acquiring sonar image...")
    result = subprocess.run(['python3', '/tmp/Scripts/Get_sonar_images.py',
                             '-o', output_folder, '-i', str(index)], 
                            capture_output=True, text=True)
    if result.returncode != 0:
        print("Error acquiring sonar image:", result.stderr)
        sys.exit(1)

def get_converted_sonar_image(output_converted_folder, output_size, range_max, index):
    # Run the script Get_converted_sonar_images.py
    print("Acquiring and converting sonar image...")
    result = subprocess.run(['python3', '/tmp/Scripts/Get_converted_sonar_images.py',
                             '-o', output_converted_folder, '--output_size', output_size, '--range_max', str(range_max), '-i', str(index)],
                            capture_output=True, text=True)
    if result.returncode != 0:
        print("Error acquiring and converting sonar image:", result.stderr)
        sys.exit(1)

def get_360_image(output_360_folder, index):
    # Run the script Get_360_images.py
    print("Acquiring 360° image...")
    result = subprocess.run(['python3', '/tmp/Scripts/Get_360_images.py',
                             '-o', output_360_folder, '-i', str(index)],
                            capture_output=True, text=True)
    if result.returncode != 0:
        print("Error acquiring 360° image:", result.stderr)
        sys.exit(1)

def modify_pose(object_name, new_pose):
    # Run the script Modify_pose.py
    print(f"Moving robot to the new pose: {new_pose}")

    # To be sure that pose parameters are separated correctly
    pose_params = new_pose.split()
    
    # Verify to have exactly 6 parameters (x, y, z, roll, pitch, yaw)
    if len(pose_params) != 6:
        print(f"Error: the pose {new_pose} doesn't have the correct format.")
        return
    command = ['python3', '/tmp/Scripts/Modify_pose.py', object_name] + pose_params

    try:
        subprocess.run(command, check=True)
        
    except subprocess.CalledProcessError as e:
        print(f"Error while modifing pose of the robot: {e}")

def process_poses(output_folder, output_converted_folder, output_pose_folder, output_size, range_max, input_file, object_name):
    global index # Use the global counter

    # Read the pose file and process it line by line
    with open(input_file, 'r') as file:
        poses = file.readlines()
    
    # For each line in the pose file
    for pose_line in poses:
        new_pose = pose_line.strip()

        # Acquire current pose
        current_pose = get_pose('flatfish')
        print("Current pose of the robot:", current_pose)

        # Move the robot to the new pose
        modify_pose(object_name, new_pose)
        print(f"Robot moved in the new pose: {new_pose}")
        time.sleep(5)

        # Save the pose in a .txt file
        if not os.path.exists(output_pose_folder):
            os.makedirs(output_pose_folder)
        file_name = f"{index}.txt"
        file_path = os.path.join(output_pose_folder, file_name)
        with open(file_path, 'w') as file:
            file.write(new_pose)
        print(f"Pose file saved in {output_pose_folder}")
        
        # Save sonar iamge
        get_sonar_image(output_folder, index)
        print(f"Sonar image saved in: {output_folder}")
        # Convert and save converted sonar image
        output_center_converted_folder = os.path.join(output_converted_folder, 'center')
        get_converted_sonar_image(output_center_converted_folder, output_size, range_max, index)
        print(f"Center sonar image converted and saved in: {output_center_converted_folder}")

        # Rotate to the left and save converted sonar image
        pose_params = new_pose.split()
        left_yaw = float(pose_params[5]) + (120 * (np.pi / 180))
        left_pose = f"{pose_params[0]} {pose_params[1]} {pose_params[2]} {pose_params[3]} {pose_params[4]} {left_yaw}"
        modify_pose(object_name, left_pose)
        time.sleep(5)
        output_left_converted_folder = os.path.join(output_converted_folder, 'left')
        get_converted_sonar_image(output_left_converted_folder, output_size, range_max, index)
        print(f"Left sonar image converted and saved in: {output_left_converted_folder}")

        # Rotate to the right and save converted sonar image
        pose_params = new_pose.split()
        right_yaw = float(pose_params[5]) - (120 * (np.pi / 180))
        right_pose = f"{pose_params[0]} {pose_params[1]} {pose_params[2]} {pose_params[3]} {pose_params[4]} {right_yaw}"
        modify_pose(object_name, right_pose)
        time.sleep(5)
        output_right_converted_folder = os.path.join(output_converted_folder, 'right')
        get_converted_sonar_image(output_right_converted_folder, output_size, range_max, index)
        print(f"Right sonar image converted and saved in: {output_right_converted_folder}")

        # Combine converted images to save 360° converted image
        get_360_image(output_converted_folder, index)
        print(f"360 converted sonar image saved in: {output_converted_folder}\n")

        index += 1

        command_left = ['rm', '-r', f'{output_converted_folder}/left']
        command_center = ['rm', '-r', f'{output_converted_folder}/center']
        command_right = ['rm', '-r', f'{output_converted_folder}/right']

        command_raw = "bash -c 'shopt -s nullglob && rm -r /tmp/SonarRawData_*'"

        try:
            subprocess.run(command_left, check=True)
            subprocess.run(command_center, check=True)
            subprocess.run(command_right, check=True)
            subprocess.run(command_raw, shell=True, check=True)
        
        except subprocess.CalledProcessError as e:
            print(f"Error while removing : {e}")

if __name__ == '__main__':
    if len(sys.argv) != 8:
        print("Expected: python3 Full_pipeline_360.py <output_folder> <output_converted_folder> <output_pose_folder> <output_size> <range_max> <object_name> <pose_file>")
        sys.exit(1)
    
    output_folder = sys.argv[1]
    output_converted_folder = sys.argv[2]
    output_pose_folder = sys.argv[3]
    output_size = sys.argv[4]
    range_max = int(sys.argv[5])
    object_name = sys.argv[6]
    pose_file = sys.argv[7]

    process_poses(output_folder, output_converted_folder, output_pose_folder, output_size, range_max, pose_file, object_name)