# Ghiotto Andrea   2118418

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import math
import numpy as np
import argparse
import tqdm
import matplotlib.pyplot as plt

# From Full_convert.py
# START

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
    parser.add_argument('-o', '--output_folder', help='Output directory that will contain converted sonar images', required=True)
    parser.add_argument('--output_size', help='Output size for the rectangular image as height,width', required=True)
    parser.add_argument('--range_max', help='Maximum range for the sonar data', type=float, required=True)
    parser.add_argument('-i', '--index', help='Index to name the image', type=int, default=0)
    parser.add_argument('--show_results', action='store_true', help='If specified, results for single images will be shown.')
    args = parser.parse_args()
    return args

# END

# Callback function to get images
def sonar_image_callback(msg, output_shape, range_max, output_dir, index):
    bridge = CvBridge()

    # Callback function to get images
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        rospy.loginfo("Sonar image received")

        # Apply CLAHE, normalization and colorization to the raw image
        cv_image = cv_image[:, :, 0]
        cv_image = cv_image.astype('uint8')
        clahe = cv2.createCLAHE()
        clahe.setClipLimit(5.0)
        clahe.setTilesGridSize((40, 40))
        clahe.apply(cv_image, cv_image)
        cv2.normalize(cv_image, cv_image,
                    -255 + 0/ 10 * 255, 255, cv2.NORM_MINMAX)
        cv_image = cv2.applyColorMap(cv_image, cv2.COLORMAP_HOT)
        
        # Convert the image to rectangular, flipped and grayscale formats
        rectangular_image = convert_to_rectangular(cv_image, range_max, output_shape)
        flipped_image = flip_image_vertically(rectangular_image)
        grayscale_image = convert_to_grayscale(flipped_image)

        if not os.path.exists(output_dir):
            os.makedirs(output_dir)

        # Use the global counter to name the image progressively
        file_path = os.path.join(output_dir, f"{index}.png")

        # Save the image
        cv2.imwrite(file_path, grayscale_image)
        rospy.loginfo(f"Image saved in {file_path}")

        # Stop the node after saving the image
        rospy.signal_shutdown("Image saved, shutting down.")

    except Exception as e:
        rospy.logerr("Error in image conversion: %s", str(e))

# Function to pass parameters to the callback
def sonar_image_callback_wrapper(msg):
    sonar_image_callback(msg, output_shape, range_max, output_dir, index)

def sonar_image_listener(output_shape, range_max, output_dir, index):
    rospy.init_node('sonar_image_listener', anonymous=True)

    # Subscribe to the topic of the sonar using wrapper function
    rospy.Subscriber("/blueview_p900/sonar_image", Image, sonar_image_callback_wrapper)

    rospy.spin()

if __name__ == "__main__":
    try:
        args = parse_args()
        output_shape = tuple(map(int, args.output_size.split(',')))
        range_max = args.range_max
        output_dir = args.output_folder
        index = args.index

        sonar_image_listener(output_shape, range_max, output_dir, index)

    except rospy.ROSInterruptException:
        pass