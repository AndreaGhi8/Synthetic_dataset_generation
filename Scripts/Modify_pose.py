# Ghiotto Andrea   2118418

import rospy
import sys
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Quaternion
import tf

def euler_to_quaternion(roll, pitch, yaw):
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    return Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])

def quaternion_to_euler(orientation):
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
    return roll, pitch, yaw

def move_robot(robot_name, position, orientation):
    model_state = ModelState()
    model_state.model_name = robot_name
    model_state.pose = Pose()

    # Set position
    model_state.pose.position.x = position[0]
    model_state.pose.position.y = position[1]
    model_state.pose.position.z = position[2]
    
    # Set orientation (converting roll, pitch and yaw to quaternion)
    roll, pitch, yaw = orientation
    model_state.pose.orientation = euler_to_quaternion(roll, pitch, yaw)
    
    # Publish the new model state to the Gazebo topic
    pub.publish(model_state)
    
    # Convert quaternion back to euler angles for the final output
    roll, pitch, yaw = quaternion_to_euler(model_state.pose.orientation)

    # Prepare the full message
    log_message = f"{robot_name} moved to\n"
    log_message += "position:\n"
    log_message += f"  x: {model_state.pose.position.x:.3f}\n"
    log_message += f"  y: {model_state.pose.position.y:.3f}\n"
    log_message += f"  z: {model_state.pose.position.z:.3f}\n"
    log_message += "orientation:\n"
    log_message += f"  roll: {roll:.3f}\n"
    log_message += f"  pitch: {pitch:.3f}\n"
    log_message += f"  yaw: {yaw:.3f}"
    
    # Print the final message
    rospy.loginfo(log_message)
    
    rospy.signal_shutdown("Pose changed. Shutting down...")

if __name__ == '__main__':
    rospy.init_node('modify_pose', anonymous=True)
    
    # Create a publisher for the topic /gazebo/set_model_state
    pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)

    # Wait for Gazebo to start and the publisher to connect
    rospy.loginfo("Waiting for Gazebo to start...")
    rospy.sleep(2)

    # Verify the correct number of passed parameters
    if len(sys.argv) != 8:
        print("Usage: Modify_pose.py <robot_name> <x> <y> <z> <roll> <pitch> <yaw>")
        sys.exit(1)
    
    robot_name = sys.argv[1]
    position = (float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]))
    orientation = (float(sys.argv[5]), float(sys.argv[6]), float(sys.argv[7]))
    
    rospy.loginfo(f"Moving {robot_name} to position {position} with orientation {orientation}")
    
    # Move the robot to the desired pose
    move_robot(robot_name, position, orientation)
    
    rospy.spin()