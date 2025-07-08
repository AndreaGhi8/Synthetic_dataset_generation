# Ghiotto Andrea   2118418

import sys
import os
import numpy as np
import math
import random
import trimesh
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def mesh_translation(mesh):
    vertices = mesh.vertices
    translation = np.array([354.449, -440.277, -4.26713])
    mesh.vertices = vertices + translation
    mesh.export('/tmp/Files/Translated_fg.stl')

def mesh_transformation(mesh):
    vertices = mesh.vertices
    translation = np.array([354.449, -440.277, -4.26713])
    rotation = np.array([[np.cos(0.385711), -np.sin(0.385711), 0], [np.sin(0.385711), np.cos(0.385711), 0], [0, 0, 1]])
    mesh.vertices = np.dot(rotation, vertices.T).T + translation
    mesh.export('/tmp/Files/Transformed_fg.stl')

def generate_random_poses(output_file, num_random_poses, mesh):
    # Starting center pose (it is the flying_garage pose)
    center_x, center_y, center_z, pitch, roll, yaw = 354.449, -440.277, -4.26713, 0.0, 0.0, 0.385711

    # Get the oriented bounding box of the mesh
    bounding_box = mesh.bounding_box_oriented

    generated_poses = 0

    with open(output_file, 'w') as f:
        while generated_poses < num_random_poses:
            x = round(random.uniform(center_x - grid_dimension/2, center_x + grid_dimension/2), 3)
            y = round(random.uniform(center_y - grid_dimension/2, center_y + grid_dimension/2), 3)

            pose = f"{x} {y} {center_z} {pitch} {roll} {yaw}"
            
            # Prepare pose for the check
            pose_values = pose.split()
            check_pose = np.array([float(value) for value in pose_values[:3]])
            check_pose = check_pose.reshape(1, 3)

            if not bounding_box.contains(check_pose):
                f.write(f"{x} {y} {center_z} {pitch} {roll} {yaw}\n")
                generated_poses += 1

def rotate_poses(file_path):
    center_x, center_y, center_z, pitch, roll, yaw = 354.449, -440.277, -4.26713, 0.0, 0.0, 0.385711

    with open(file_path, 'r') as file:
        lines = file.readlines()
    
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    
    poses = []

    for line in lines:
        x, y, z, pitch, roll, yaw = map(float, line.split())
        
        x_traslated = x - center_x
        y_traslated = y - center_y
        x_new = cos_yaw * x_traslated - sin_yaw * y_traslated
        y_new = sin_yaw *x_traslated + cos_yaw * y_traslated
        x_new += center_x
        y_new += center_y

        yaw = calculate_orientation(x_new, y_new, center_x, center_y)
        random_yaw = random.uniform(-30, 30)
        if random_yaw < 0:
            yaw = float(yaw) + (random_yaw * (np.pi / 180))
        else:
            yaw = float(yaw) - (random_yaw * (np.pi / 180))

        poses.append(f"{x_new:.6f} {y_new:.6f} {z:.6f} {pitch:.6f} {roll:.6f} {yaw:.6f}\n")

    with open(file_path, 'w') as file:
        file.writelines(poses)

# For each point of the grid, compute yaw in order to always have the sonar pointing to the center of the grid
def calculate_orientation(x, y, center_x, center_y):
    dx = center_x - x
    dy = center_y - y
    yaw = math.atan2(dy, dx) # function atan2 to compute yaw between current point and the center of the grid
    return yaw

def read_points(file_path):
    x = []
    y = []
    z = []

    with open(file_path, 'r') as file:
        for line in file:
            l = line.split()
            if len(l) >= 6:
                x.append(float(l[0]))
                y.append(float(l[1]))
                z.append(float(l[2]))
    
    return np.array(x), np.array(y), np.array(z)

def visualize_points(x1, y1, z1, x2, y2, z2, mesh):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Visualize all the random points
    ax.scatter(x1, y1, z1, c='b', marker='o', label='VALIDATION POINT')
    ax.scatter(x2, y2, z2, c='r', marker='o', label='TEST POINT')

    # Visualize the mesh
    vertices = mesh.vertices
    faces = mesh.faces
    ax.plot_trisurf(vertices[:, 0], vertices[:, 1], vertices[:, 2], triangles=faces, color='gray', alpha=0.5, linewidth=0.5)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D points')
    ax.legend()

    plt.show()

if __name__ == "__main__":

    if len(sys.argv) != 4:
        print("Expected: python3 Generate_random_poses.py <folder> <grid_dimension> <num_random_poses>")
        sys.exit(1)
    
    folder = sys.argv[1]
    grid_dimension = int(sys.argv[2])
    num_random_poses = int(sys.argv[3])
    output_file_validation = os.path.join(folder, "Validation_poses.txt")
    output_file_test = os.path.join(folder, "Test_poses.txt")

    mesh = trimesh.load_mesh('~/uuv_ws/src/dave/models/dave_object_models/models/flying_garage/collision.stl')
    mesh_translation(mesh)
    translated_mesh = trimesh.load('/tmp/Files/Translated_fg.stl')

    generate_random_poses(output_file_validation, num_random_poses, translated_mesh)
    generate_random_poses(output_file_test, num_random_poses, translated_mesh)

    mesh = trimesh.load_mesh('~/uuv_ws/src/dave/models/dave_object_models/models/flying_garage/collision.stl')
    mesh_transformation(mesh)
    transformed_mesh = trimesh.load('/tmp/Files/Transformed_fg.stl')
    
    rotate_poses(output_file_validation)
    rotate_poses(output_file_test)
    x1, y1, z1 = read_points(output_file_validation)
    x2, y2, z2 = read_points(output_file_test)
    visualize_points(x1, y1, z1, x2, y2, z2, transformed_mesh)