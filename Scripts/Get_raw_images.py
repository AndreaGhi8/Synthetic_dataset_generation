# Ghiotto Andrea   2118418

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import argparse
from functools import partial

def parse_args():
    parser = argparse.ArgumentParser(description='Raw image acquiring tool')
    parser.add_argument('-o', '--output_folder', help='Output directory that will contain raw images', required=True)
    parser.add_argument('-i', '--index', help='Index to name the image', type=int, default=0)
    args = parser.parse_args()
    return args

# Callback function to get raw images
def sonar_image_callback(msg, output_dir, index):
    bridge = CvBridge()
    
    # Convert the ROS message into a format compatible with OpenCV
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        rospy.loginfo("Raw image received")

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
        rospy.logerr("Error while getting raw image: %s", str(e))

def raw_image_listener(output_dir, index):
    rospy.init_node('raw_image_listener', anonymous=True)
    
    # Subscribe to the topic of the sonar, using functools.partial to pass output_dir to the callback
    rospy.Subscriber("/blueview_p900/sonar_image", Image, partial(sonar_image_callback, output_dir=output_dir, index=index))
    
    rospy.spin()

if __name__ == '__main__':
    try:
        args = parse_args()
        output_dir = args.output_folder
        index = args.index
        
        raw_image_listener(output_dir, index)
    except rospy.ROSInterruptException:
        pass