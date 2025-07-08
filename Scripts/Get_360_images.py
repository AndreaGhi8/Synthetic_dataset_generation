# Ghiotto Andrea   2118418

import os
import cv2
import numpy as np
import argparse

# To get images from subfolders
def load_images_from_folders(output_dir):
    left_folder = os.path.join(output_dir, "left")
    center_folder = os.path.join(output_dir, "center")
    right_folder = os.path.join(output_dir, "right")

    if not os.path.exists(left_folder) or not os.path.exists(center_folder) or not os.path.exists(right_folder):
        raise FileNotFoundError("One or more folder 'left', 'center' or 'right' doesn't exist.")
    
    # Read images
    left_image = cv2.imread(os.path.join(left_folder, os.listdir(left_folder)[0]))
    center_image = cv2.imread(os.path.join(center_folder, os.listdir(center_folder)[0]))
    right_image = cv2.imread(os.path.join(right_folder, os.listdir(right_folder)[0]))

    return left_image, center_image, right_image

# To concatenate images orizontally
def concatenate_images(left_img, center_img, right_img):
    if left_img.shape[0] == center_img.shape[0] == right_img.shape[0]:
        concatenated_image = np.concatenate((left_img, center_img, right_img), axis=1)
        return concatenated_image
    else:
        raise ValueError("Images must be the same height to be concatenated.")

# To save concatenated image
def save_concatenated_image(concatenated_image, output_dir, index):
    # Use the global counter to named the image progressively
    file_path = os.path.join(output_dir, f"{index}.png")

    # Save the image
    cv2.imwrite(file_path, concatenated_image)
    print(f"360° concatenated image saved in {file_path}")

# To get the 360° concatenated image
def get_360_image(output_dir, index):
    try:
        # Load images
        left_image, center_image, right_image = load_images_from_folders(output_dir)
        if left_image is None or center_image is None or right_image is None:
            raise ValueError("An image has not been loaded correctly.")
        
        # Concatenate images
        concatenated_image = concatenate_images(left_image, center_image, right_image)
        
        # Save 360° concatenate image
        save_concatenated_image(concatenated_image, output_dir, index)

    except Exception as e:
        print(f"Error during the acquiring of the 360° concatenated image: {e}")

def parse_args():
    parser = argparse.ArgumentParser(description='Concatenation of images to get a 360° image')
    parser.add_argument('-o', '--output_folder', help='Output folder with subfolders', required=True)
    parser.add_argument('-i', '--index', help='Index to name the image', type=int, default=0)
    args = parser.parse_args()
    return args

if __name__ == "__main__":
    try:
        args = parse_args()
        output_dir = args.output_folder
        index = args.index
        get_360_image(output_dir, index)
    except Exception as e:
        print(f"Error: {e}")