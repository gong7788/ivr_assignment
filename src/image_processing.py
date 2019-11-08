#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import numpy as np
import message_filters
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError

class image_converter:
    # Defines publisher and subscriber
    def __init__(self):
        # initialize the node named image_processing
        rospy.init_node('listener', anonymous=True)

        self.target_position_xz = None
        self.target_position_yz = None

        self.joints_pub = rospy.Publisher("joints_pos", Float64MultiArray, queue_size=10)
        self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
        self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
        self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
        self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)

        self.XZ_position = message_filters.Subscriber("/camera1/target_position", Float64MultiArray)
        self.YZ_position = message_filters.Subscriber("/camera2/target_position", Float64MultiArray)
        self.sync = message_filters.ApproximateTimeSynchronizer([self.XZ_position, self.YZ_position], 10, 0.1, allow_headerless=True)
        self.sync.registerCallback(self.callback)

    def callback(self, xz_pos, yz_pos):
        print('xz', xz_pos.data)
        print('yz', yz_pos.data)

        #control the robot joints
        # self.joint1 = Float64()
        # self.joint1.data = 0.0
        # self.joint2 = Float64()
        # self.joint2.data = 1
        # self.joint3 = Float64()
        # self.joint3.data = 1
        # self.joint4 = Float64()
        # self.joint4.data = 1

        # Publish the results
        # try:
        #     self.robot_joint1_pub.publish(self.joint1)
        #     self.robot_joint2_pub.publish(self.joint2)
        #     self.robot_joint3_pub.publish(self.joint3)
        #     self.robot_joint4_pub.publish(self.joint4)
        # except CvBridgeError as e:
        #     print(e)


# call the class
def main(args):
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)