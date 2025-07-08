# Ghiotto Andrea   2118418

import cv2
import os
import shutil
import argparse

def get_image_paths(base_path, step=5):
    return [os.path.join(base_path, f"{i}.png") for i in range(0, 31375, step)]

def move_files_to_delete(base_paths, deleted_paths, index):
    for i in range(index, index + 5):
        for src_path, dest_path in zip(base_paths, deleted_paths):
            file_extension = ".png" if "poses" not in src_path else ".txt"
            file_src = os.path.join(src_path, f"{i}{file_extension}")
            file_dest = os.path.join(dest_path, os.path.basename(file_src))
            
            if os.path.exists(file_src):
                shutil.move(file_src, file_dest)
            else:
                print(f"File not found: {file_src}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Image filtering")
    parser.add_argument("base_dir", type=str, help="Path to the base directory containing images folders")
    args = parser.parse_args()

    base_dir = args.base_dir
    img_folder = os.path.join(base_dir, "imgs")
    sub_dirs = ["imgs", "raw_imgs", "fan_imgs", "poses"]
    base_paths = [os.path.join(base_dir, sub_dir) for sub_dir in sub_dirs]
    
    deleted_dir = os.path.join(base_dir, "deleted_images")
    if not os.path.exists(deleted_dir):
        os.makedirs(deleted_dir)
    deleted_paths = [os.path.join(deleted_dir, sub_dir) for sub_dir in sub_dirs]
    for path in deleted_paths:
        if not os.path.exists(path):
            os.makedirs(path)

    img_paths = get_image_paths(img_folder)
    for img_path in img_paths:
        if not os.path.exists(img_path):
            continue

        img = cv2.imread(img_path)
        if img is None:
            continue

        window_name = os.path.basename(img_path)
        cv2.imshow(window_name, img)

        while True:
            key = cv2.waitKey(100) & 0xFF  # Wait for keyboard input

            if key == ord('k'):  # Keep the image
                print(f"Image OK: {window_name}")
                break
            elif key == ord('r'):  # Move the image and the following 4
                img_index = int(os.path.basename(img_path).split('.')[0])
                move_files_to_delete(base_paths, deleted_paths, img_index)
                print(f"Image moved: {window_name}")
                break

        cv2.destroyAllWindows()