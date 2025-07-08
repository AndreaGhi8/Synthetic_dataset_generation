# Ghiotto Andrea   2118418

import sys
import os
import numpy as np
import trimesh
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

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

    if len(sys.argv) != 2:
        print("Expected: python3 Generate_random_poses.py <folder>")
        sys.exit(1)
    
    folder = sys.argv[1]
    output_file_validation = os.path.join(folder, "Validation_poses.txt")
    output_file_test = os.path.join(folder, "Test_poses.txt")

    transformed_mesh = trimesh.load('/tmp/Files/Transformed_fg.stl')

    x1, y1, z1 = read_points(output_file_validation)
    x2, y2, z2 = read_points(output_file_test)
    visualize_points(x1, y1, z1, x2, y2, z2, transformed_mesh)