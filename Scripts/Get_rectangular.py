# Ghiotto Andrea   2118418

import os
import sys
import math
import numpy as np
import cv2

def get_sonar_parameters():
    view_angle = 120
    bin_count = 150
    beam_count = 256
    return view_angle, bin_count, beam_count

def convert_to_rectangular(image, range_max, output_shape):
    image_height, image_width, _ = image.shape
    view_angle, bin_count, beam_count = get_sonar_parameters()

    resolution = range_max / bin_count
    dist_to_pixel = image_height / range_max
    angle_resolution = view_angle / beam_count

    origin_x = image_width / 2
    origin_y = image_height - 1

    start_angle = 180 - (180 - view_angle) / 2
    end_angle = (180 - view_angle) / 2
    start_angle_rad = math.radians(start_angle)
    end_angle_rad = math.radians(end_angle)
    angle_resolution_rad = math.radians(angle_resolution)

    rectangular_height, rectangular_width = output_shape

    rectangular_image = np.zeros((rectangular_height, rectangular_width, 3), dtype=np.uint8)

    for beam in range(rectangular_width):
        for rangee in range(rectangular_height):
            theta = start_angle_rad - angle_resolution_rad / 2 - beam * angle_resolution_rad

            bin = rectangular_height - rangee
            bin_to_meters = bin * resolution
            meters_to_pixel = bin_to_meters * dist_to_pixel

            pixel_x = round(origin_x + meters_to_pixel * math.cos(theta))
            pixel_y = round(origin_y - meters_to_pixel * math.sin(theta))

            if 0 <= pixel_x < image_width and 0 <= pixel_y < image_height:
                rectangular_image[rangee, beam] = image[pixel_y, pixel_x]

    return rectangular_image

def process_images(input_folder, output_folder, range_max, output_shape):
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    for i in range(31375):
        image_filename = os.path.join(input_folder, f"{i}.png")
        
        if not os.path.exists(image_filename):
            continue
        
        # Carica l'immagine
        image = cv2.imread(image_filename)
        if image is None:
            continue

        rectangular_image = convert_to_rectangular(image, range_max, output_shape)

        output_image_filename = os.path.join(output_folder, f"{i}.png")
        cv2.imwrite(output_image_filename, rectangular_image)


if __name__ == '__main__':
    if len(sys.argv) != 5:
        print("Usage: python3 Get_rectangular.py <input_folder> <output_folder> <range_max> <output_shape>")
        sys.exit(1)
    
    input_folder = sys.argv[1]
    output_folder = sys.argv[2]
    range_max = int(sys.argv[3])
    output_shape = tuple(map(int, sys.argv[4].split(',')))
    
    process_images(input_folder, output_folder, range_max, output_shape)