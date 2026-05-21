#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Float32MultiArray

def info_callback(msg):
    # msg.data contains the [x, y, score] we packed earlier
    if len(msg.data) >= 3:
        vessel_x = msg.data[0]
        vessel_y = msg.data[1]
        vessel_score = msg.data[2]

        # If the score is 0, it means no vessel was detected
        if vessel_score == 0:
            rospy.logwarn("No vessel targets were detected!")
        else:
            rospy.loginfo("Find vessel! central coordinate X: {:.1f}, Y: {:.1f} | score: {:.2f}".format(
                vessel_x, vessel_y, vessel_score))

            # TODO: Add your robotic arm control logic here
            # For example: if x < 240, it means the vessel is to the left, move the arm to the left...

def listener():
    # Initialize a clean ROS node
    rospy.init_node('vessel_decision_node', anonymous=True)

    # Subscribe to the /vessel/info topic
    rospy.Subscriber('/vessel/info', Float32MultiArray, info_callback)

    rospy.loginfo("Decision Node Started. Waiting for vessel data...")
    rospy.spin()

if __name__ == '__main__':
    listener()
