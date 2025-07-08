# Ghiotto Andrea   2118418

import subprocess
import sys
import time
import os

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

def get_raw_image(output_folder, index):
    # Run the script Get_raw_images.py
    print("Acquiring raw image...")
    result = subprocess.run(['python3', '/tmp/Scripts/Get_raw_images.py',
                             '-o', output_folder, '-i', str(index)], 
                            capture_output=True, text=True)
    if result.returncode != 0:
        print("Error acquiring raw image:", result.stderr)
        sys.exit(1)
    
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

def process_poses(output_raw_folder, output_folder, output_converted_folder, output_pose_folder, output_size, range_max, object_name, trasnformed_pose_file, pose_file):
    global index # Use the global counter

    # Read files and process them line by line
    with open(trasnformed_pose_file, 'r') as file:
        transformed_poses = file.readlines()

    with open(pose_file, 'r') as file:
        poses = file.readlines()
    
    # For each line in the transformed poses file (file with poses w.r.t. sonar)
    for pose_line in transformed_poses:
        new_pose = pose_line.strip()

        # Acquire current pose
        current_pose = get_pose(object_name)
        print("Current pose of the robot:", current_pose)

        # Move the robot to the new pose
        modify_pose(object_name, new_pose)
        print(f"Robot moved in the new pose: {new_pose}")
        time.sleep(5)

        # Save the original pose of the robot's cog in a .txt file
        if not os.path.exists(output_pose_folder):
            os.makedirs(output_pose_folder)
        file_name = f"{index}.txt"
        file_path = os.path.join(output_pose_folder, file_name)
        pose_to_save = poses[index].strip()
        with open(file_path, 'w') as file:
            file.write(pose_to_save)
        print(f"Pose file saved in {output_pose_folder}")

        # Save raw image
        get_raw_image(output_raw_folder, index)
        print(f"Raw image saved in: {output_raw_folder}")
        
        # Save sonar iamge
        get_sonar_image(output_folder, index)
        print(f"Sonar image saved in: {output_folder}")
        
        # Convert and save converted sonar image
        get_converted_sonar_image(output_converted_folder, output_size, range_max, index)
        print(f"Sonar image converted and saved in: {output_converted_folder}")

        index += 1

        command = "bash -c 'shopt -s nullglob && rm -r /tmp/SonarRawData_*'"

        try:
            subprocess.run(command, shell=True, check=True)

        except subprocess.CalledProcessError as e:
            print(f"Error while removing : {e}")

if __name__ == '__main__':
    if len(sys.argv) != 10:
        print("Expected: python3 Full_pipeline.py <output_raw_folder> <output_folder> <output_converted_folder> <output_pose_folder> <output_size> <range_max> <object_name> <transformed_pose_file> <pose_file>")
        sys.exit(1)
    
    output_raw_folder = sys.argv[1]
    output_folder = sys.argv[2]
    output_converted_folder = sys.argv[3]
    output_pose_folder = sys.argv[4]
    output_size = sys.argv[5]
    range_max = int(sys.argv[6])
    object_name = sys.argv[7]
    transformed_pose_file = sys.argv[8]
    pose_file = sys.argv[9]

    process_poses(output_raw_folder, output_folder, output_converted_folder, output_pose_folder, output_size, range_max, object_name, transformed_pose_file, pose_file)