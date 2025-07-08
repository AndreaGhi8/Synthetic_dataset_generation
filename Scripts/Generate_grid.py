# Ghiotto Andrea   2118418

import math
import sys
import os
import trimesh
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def mesh_transformation(mesh):
    vertices = mesh.vertices
    translation = np.array([354.449, -440.277, -4.26713])
    rotation = np.array([[np.cos(0.385711), -np.sin(0.385711), 0], [np.sin(0.385711), np.cos(0.385711), 0], [0, 0, 1]])
    mesh.vertices = np.dot(rotation, vertices.T).T + translation
    mesh.export('/tmp/Files/Transformed_fg.stl')

# For each point of the grid, compute yaw in order to always have the sonar pointing to the center of the grid
def calculate_orientation(x, y, center_x, center_y):
    dx = center_x - x
    dy = center_y - y
    yaw = math.atan2(dy, dx) # function atan2 to compute yaw between current point and the center of the grid
    return yaw

def generate_grid(grid_size, grid_resolution, output_grid_folder, mesh):
    # Starting center pose (it is the flying_garage pose)
    center_x, center_y, center_z, pitch, roll, yaw = 354.449, -440.277, -4.26713, 0.0, 0.0, 0.385711

    num_points = int(grid_size / grid_resolution) + 1  # number of points for each row/coloumn of the grid

    # Get the oriented bounding box of the mesh
    bounding_box = mesh.bounding_box_oriented

    grid_poses = []
    wrong_poses = []

    for i in range(num_points):
        for j in range(num_points):
            # Compute x and y coordinates of each point of the grid (z coordinate is always the same)
            x = center_x + math.cos(yaw) * ((i - num_points // 2) * grid_resolution) - math.sin(yaw) * ((j - num_points // 2) * grid_resolution)
            y = center_y + math.sin(yaw) * ((i - num_points // 2) * grid_resolution) + math.cos(yaw) * ((j - num_points // 2) * grid_resolution)
            new_yaw = calculate_orientation(x, y, center_x, center_y)

            new_pose = f"{x} {y} {center_z} {pitch} {roll} {new_yaw}"

            # Prepare pose for the check
            pose_values = new_pose.split()
            check_pose = np.array([float(value) for value in pose_values[:3]])
            check_pose = check_pose.reshape(1, 3)

            if not bounding_box.contains(check_pose):
                grid_poses.append(new_pose)
            else:
                wrong_poses.append(new_pose)
    
    if not os.path.exists(output_grid_folder):
        os.makedirs(output_grid_folder)

    output_file = os.path.join(output_grid_folder, "Grid_poses.txt")
    in_output_file = os.path.join(output_grid_folder, "Wrong_poses.txt")

    with open(output_file, 'w') as f:
        for pose in grid_poses:
            f.write(pose + "\n")
    with open(in_output_file, 'w') as f:
        for pose in wrong_poses:
            f.write(pose + "\n")

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

    # Visualize all the points of the grid
    ax.scatter(x1, y1, z1, c='b', marker='o', label='OK POINT')
    ax.scatter(x2, y2, z2, c='r', marker='o', label='WRONG POINT')

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
        print("Expected: python3 Generate_grid.py <grid_dimension> <grid_resolution> <output_grid_folder>")
        sys.exit(1)

    mesh = trimesh.load_mesh('~/uuv_ws/src/dave/models/dave_object_models/models/flying_garage/collision.stl')
    mesh_transformation(mesh)
    transformed_mesh = trimesh.load('/tmp/Files/Transformed_fg.stl')

    grid_size = int(sys.argv[1])
    grid_resolution = float(sys.argv[2])
    output_grid_folder = sys.argv[3]
    generate_grid(grid_size, grid_resolution, output_grid_folder, transformed_mesh)

    file_path_1 = '/tmp/Files/Grid_poses.txt'
    file_path_2 = '/tmp/Files/Wrong_poses.txt'
    x1, y1, z1 = read_points(file_path_1)
    x2, y2, z2 = read_points(file_path_2)
    visualize_points(x1, y1, z1, x2, y2, z2, mesh)