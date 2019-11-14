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

        self.target_pub = rospy.Publisher('/target/detected', Float64MultiArray, queue_size=10)

        self.joints_pub = rospy.Publisher("joints_pos", Float64MultiArray, queue_size=10)
        self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
        self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
        self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
        self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)

        self.XZ_position = message_filters.Subscriber("/camera2/target_position", Float64MultiArray)
        self.YZ_position = message_filters.Subscriber("/camera1/target_position", Float64MultiArray)
        self.sync = message_filters.ApproximateTimeSynchronizer([self.XZ_position, self.YZ_position], 10, 0.1, allow_headerless=True)
        self.sync.registerCallback(self.callback)

    #TODO finds joints
    #TODO transfer to meters coordinate


    def find_angles(self, centers):
        """Finds joints angles
           The robot is redundant, so multiple answers for joints, ignore joint2
            :param xz_pos: list, 5 positions in xz plane [red, green, blue, yellow, target]
            :param yz_pos: list, 5 positions in xz plane
            :return angles
        """
        # centers = [red, green, blue, yellow, target] -- red:[x,y,z] ...
        red_true = centers[0]
        green_true = centers[1]
        blue_true = centers[2]

        a = [0, 0, 3, 2]
        alpha = [1.57, 1.57, 1.57, 0]
        d = [2, 0, 0, 0]




    def target_coordinates(self, target, base, blue):
        """Transforms pixel coordinates into meters

            :param target: [x,y,z] target coordinates in pixel
            :param base:   [x,y,z] base(yellow joint) coordinates in pixel
            :param blue:   [x,y,z] blue joint coordinates in pixel
            :return        [x,y,z] coordinates in meters
        """
        distance = np.linalg.norm(blue - base)
        target_base = (2 / distance) * (target - base)
        target_base[2] = -target_base[2]
        return target_base




    def callback(self, xz_pos, yz_pos):
        centers = []
        for i in range(5):
            x = xz_pos.data[2 * i]
            y = yz_pos.data[2 * i]
            z = (xz_pos.data[2 * i + 1] + yz_pos.data[2 * i + 1]) / 2
            centers.append(np.array([x, y, z]))

        joint1 = self.find_angles(centers)[0]
        joint2 = self.find_angles(centers)[1]
        joint3 = self.find_angles(centers)[2]
        joint4 = self.find_angles(centers)[3]
        # print('joint1:{:.2f}, joint2:{:.2f}, joint3:{:.2f}, joint4:{:.2f}'.format(joint1, joint2, joint3, joint4))
        blue = centers[-3]
        yellow = centers[-2]
        target = centers[-1]
        # print(self.target_coordinates(target, yellow, blue))
        target_pub = Float64MultiArray()
        target_pub.data = self.target_coordinates(target, yellow, blue)

        print("Red: ", self.target_coordinates(centers[0], yellow, blue))
        print("Green: ", self.target_coordinates(centers[1], yellow, blue))
        print("Blue: ", self.target_coordinates(centers[2], yellow, blue))
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
        try:
            # self.robot_joint1_pub.publish(self.joint1)
            # self.robot_joint2_pub.publish(self.joint2)
            # self.robot_joint3_pub.publish(self.joint3)
            # self.robot_joint4_pub.publish(self.joint4)
            self.target_pub.publish(target_pub)
        except CvBridgeError as e:
            print(e)



    # def rotation(self, alpha, a, d, theta, yellow):
    #     Transform = []
    #     for i in range(4):
    #         T_theta = np.array([np.cos(theta[i]), -np.sin(theta[i]), 0, 0],
    #                            [np.sin(theta[i]), np.cos(theta[i]), 0, 0],
    #                            [0, 0, 1, 0],
    #                            [0, 0, 0, 1])
    #         T_d = np.array([1, 0, 0, 0],
    #                        [0, 1, 0, 0],
    #                        [0, 0, 1, d[i]],
    #                        [0, 0, 0, 1])
    #         T_a = np.array([1, 0, 0, a[i]],
    #                        [0, 1, 0, 0],
    #                        [0, 0, 1, 0],
    #                        [0, 0, 0, 1])
    #         T_alpha = np.array([1, 0, 0, 0],
    #                            [0, np.cos(theta[i]), -np.sin(theta[i]), 0],
    #                            [0,  np.sin(theta[i]), np.cos(theta[i]), 0],
    #                            [0, 0, 0, 1])
    #         Transform.append(T_theta.dot(T_d.dot(T_a.dot(T_alpha))))
    #         return Transform[0].dot(Transform[1].dot(Transform[2].dot(Transform[3])))





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