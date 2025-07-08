# Ghiotto Andrea   2118418

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
import tf
import argparse
from functools import partial

pose = None

def parse_args():
    parser = argparse.ArgumentParser(description='Get pose of an object')
    parser.add_argument('-n', '--object_name', help='Name of the object', required=True)
    args = parser.parse_args()
    return args

# Callback for the topic of the pose (gazebo/model_states)
def pose_callback(msg, object_name):
    global pose
    if object_name in msg.name:
        robot_idx = msg.name.index(object_name)
        pose = msg.pose[robot_idx]
        if pose:
            # Prepare one string output
            log_message = f"{object_name}\n"
            log_message += "position:\n"
            log_message += f"  x: {pose.position.x:.3f}\n"
            log_message += f"  y: {pose.position.y:.3f}\n"
            log_message += f"  z: {pose.position.z:.3f}\n"
            
            # Convert quaternion to Euler angles (roll, pitch, yaw)
            euler_angles = tf.transformations.euler_from_quaternion([
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w
            ])
            roll, pitch, yaw = euler_angles
            
            # Add orientation to the one string output
            log_message += "orientation:\n"
            log_message += f"  roll: {roll:.3f}\n"
            log_message += f"  pitch: {pitch:.3f}\n"
            log_message += f"  yaw: {yaw:.3f}\n"
            
            # Print one string output
            rospy.loginfo(log_message)

        rospy.signal_shutdown("Pose received and printed. Shutting down...")

if __name__ == '__main__':
    args = parse_args()
    object_name = args.object_name

    rospy.init_node('get_pose', anonymous=True)

    # Subscribe to the topic /gazebo/model_states
    rospy.Subscriber("/gazebo/model_states", ModelStates, partial(pose_callback, object_name=object_name))

    rospy.spin()