import os
import cv2
import math
import numpy as np
import argparse
import tqdm
import matplotlib.pyplot as plt

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


def flip_image_vertically(image):
    return cv2.flip(image, 1)


def convert_to_grayscale(image):
    return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


def parse_args():
    parser = argparse.ArgumentParser(description='Sonar image processing tool')
    parser.add_argument('-i', '--input_folder', help='Input directory containing the fan dataset', required=True)
    parser.add_argument('-o', '--output_folder', help='Output directory that will contain the processed dataset', required=True)
    parser.add_argument('--output_size', help='Output size for the rectangular image as height,width', required=True)
    parser.add_argument('--range_max', help='Maximum range for the sonar data', type=float, required=True)
    parser.add_argument('--show_results', action='store_true', help='If specified, results for single images will be shown.')
    args = parser.parse_args()
    return args


def process_images_in_folder(input_folder, output_folder, output_shape, range_max, show_results):
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    for filename in tqdm.tqdm(os.listdir(input_folder)):
        if filename.endswith(('.png', '.jpg', '.jpeg')):
            input_image_path = os.path.join(input_folder, filename)
            output_image_path = os.path.join(output_folder, filename)

            input_image = cv2.imread(input_image_path, cv2.IMREAD_COLOR)

            rectangular_image = convert_to_rectangular(input_image, range_max, output_shape)

            flipped_image = flip_image_vertically(rectangular_image)

            grayscale_image = convert_to_grayscale(flipped_image)

            cv2.imwrite(output_image_path, grayscale_image)

            if show_results:
                _, axs = plt.subplots(1, 3)
                axs[0].title.set_text("Original")
                axs[1].title.set_text("Rectangular & Flipped")
                axs[2].title.set_text("Grayscale")
                axs[0].imshow(cv2.cvtColor(input_image, cv2.COLOR_BGR2RGB))
                axs[1].imshow(cv2.cvtColor(flipped_image, cv2.COLOR_BGR2RGB))
                axs[2].imshow(grayscale_image, cmap='gray')
                plt.show()

if __name__ == "__main__":
    args = parse_args()
    output_shape = tuple(map(int, args.output_size.split(',')))
    process_images_in_folder(args.input_folder, args.output_folder, output_shape, args.range_max, args.show_results)