import rospy
from sensor_msgs.msg import JointState
import math

# Global variable to store previous joint values
previous_joint_values = None

# Callback function to handle received joint states
def joint_states_callback(msg):
    global previous_joint_values

    try:
        # Extract values of joint1 to joint5 in radians and convert to degrees
        joint1_degrees = math.degrees(msg.position[msg.name.index('joint1')])
        joint2_degrees = math.degrees(msg.position[msg.name.index('joint2')])
        joint3_degrees = math.degrees(msg.position[msg.name.index('joint3')])
        joint4_degrees = math.degrees(msg.position[msg.name.index('joint4')])
        joint5_degrees = math.degrees(msg.position[msg.name.index('joint5')])

        # Round values to two decimal places
        joint1_degrees = round(joint1_degrees, 2)
        joint2_degrees = round(joint2_degrees, 2)
        joint3_degrees = round(joint3_degrees, 2)
        joint4_degrees = round(joint4_degrees, 2)
        joint5_degrees = round(joint5_degrees, 2)

        # Check if current joint values are different from previous ones
        current_joint_values = (joint1_degrees, joint2_degrees, joint3_degrees, joint4_degrees, joint5_degrees)
        if current_joint_values != previous_joint_values:
            # Write to jog.tap file
            with open('jog.tap', 'w') as file:
                file.write("F800\n")
                file.write("G90 ")
                file.write("X {:.2f} Y {:.2f} Z {:.2f} A {:.2f} B {:.2f} C {:.2f}".format(joint1_degrees, joint2_degrees, joint3_degrees, joint4_degrees, joint5_degrees, 0.0))
            # Update previous joint values
            previous_joint_values = current_joint_values
        else:
            # If the file has been written, shut down the ROS node
            rospy.signal_shutdown("G-code conversion completed.")

    except ValueError:
        rospy.logwarn("Could not find all required joint names in message.")

def listener():
    global previous_joint_values
    rospy.init_node('joint_listener', anonymous=True)
    rospy.Subscriber("/joint_states", JointState, joint_states_callback)
    # Initialize previous joint values with None
    previous_joint_values = None
    rospy.spin()

if __name__ == '__main__':
    listener()

