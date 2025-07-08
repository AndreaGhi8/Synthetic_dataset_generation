# Ghiotto Andrea   2118418

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import argparse
from functools import partial

def parse_args():
    parser = argparse.ArgumentParser(description='Sonar image processing tool')
    parser.add_argument('-o', '--output_folder', help='Output directory that will contain sonar images', required=True)
    parser.add_argument('-i', '--index', help='Index to name the image', type=int, default=0)
    args = parser.parse_args()
    return args

# Callback function to get images
def sonar_image_callback(msg, output_dir, index):
    bridge = CvBridge()
    
    # Convert the ROS message into a format compatible with OpenCV
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
                    -255 + 0 / 10 * 255, 255, cv2.NORM_MINMAX)
        cv_image = cv2.applyColorMap(cv_image, cv2.COLORMAP_HOT)

        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
        
        # Use the global counter to name the image progressively
        file_path = os.path.join(output_dir, f"{index}.png")
        
        # Save the image
        cv2.imwrite(file_path, cv_image)
        rospy.loginfo(f"Image saved in {file_path}")
        
        # Stop the node after saving the image
        rospy.signal_shutdown("Image saved, shutting down.")
    
    except Exception as e:
        rospy.logerr("Error while getting sonar image: %s", str(e))

def sonar_image_listener(output_dir, index):
    rospy.init_node('sonar_image_listener', anonymous=True)
    
    # Subscribe to the topic of the sonar, using functools.partial to pass output_dir to the callback
    rospy.Subscriber("/blueview_p900/sonar_image", Image, partial(sonar_image_callback, output_dir=output_dir, index=index))
    
    rospy.spin()

if __name__ == '__main__':
    try:
        args = parse_args()
        output_dir = args.output_folder
        index = args.index
        
        sonar_image_listener(output_dir, index)
    except rospy.ROSInterruptException:
        pass