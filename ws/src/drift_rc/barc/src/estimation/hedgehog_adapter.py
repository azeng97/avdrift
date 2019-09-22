#!/usr/bin/env python
import rospy
from marvelmind.msg import hedge_pos_ang
from sensor_msgs.msg import

# Create a callback function for the subscriber.
def callback(data):
    # Simply print out values in our custom message.
    rospy.loginfo()
    rospy.loginfo()

# This ends up being the main while loop.
def listener():
    # Get the ~private namespace parameters from command line or launch file.
    # Create a subscriber with appropriate topic, custom message and name of callback function.
    rospy.Subscriber("/hedge_pos_ang", hedge_pos_ang, callback)
    # Wait for messages on topic, go to callback function when new messages arrive.
    rospy.spin()

# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('hedge_adapter', anonymous = True)
    # Go to the main loop.
    listener()