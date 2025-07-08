# Ghiotto Andrea   2118418

import os
import subprocess
import shlex
import argparse

def parse_args():
    parser = argparse.ArgumentParser(description='Copy .jpg files from a Docker container')
    parser.add_argument('--container_name', help='Docker container name', required=True)
    parser.add_argument('--source_path', help='Path inside the container to search for .jpg files', required=True)
    parser.add_argument('--dest_path', help='Local destination path to copy files to', required=True)
    return parser.parse_args()
    
if __name__ == "__main__":
    args = parse_args()
    container_name = args.container_name
    source_path = args.source_path
    dest_path = os.path.expanduser(args.dest_path)

    # Create the destination folder if it doesn't exist
    os.makedirs(dest_path, exist_ok=True)

    # Find .jpg files in the source path inside the container and get a list of them
    find_command = f"sudo docker exec {container_name} find {source_path} -type f -name '*.jpg'"
    result = subprocess.run(find_command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

    # If the list is valid, copy the files
    if result.returncode == 0:
        files = result.stdout.strip().split("\n")

        # Copy each .jpg file
        for file in files:
            # Extract just the file name
            file_name = file.split("/")[-1]

            # Use shlex.quote to properly escape the file path
            file_escaped = shlex.quote(file)
            dest_path_escaped = shlex.quote(f"{dest_path}/{file_name}")

            copy_command = f"sudo docker cp {container_name}:{file_escaped} {dest_path_escaped}"
            subprocess.run(copy_command, shell=True, check=True)

        print("Copy completed!")
    else:
        print(f"Error while executing the 'find' command!")