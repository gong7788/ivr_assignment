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

        self.XZ_position = message_filters.Subscriber("/camera2/target_position", Float64MultiArray)
        self.YZ_position = message_filters.Subscriber("/camera1/target_position", Float64MultiArray)
        self.sync = message_filters.ApproximateTimeSynchronizer([self.XZ_position, self.YZ_position], 10, 0.1, allow_headerless=True)
        self.sync.registerCallback(self.callback)

    #TODO finds joints
    #TODO transfer to meters coordinate

    def angle_between_vectors(self, vector1, vector2):
        """Finds angles between two vectors

            :param: vector1: first vector
            :param: vector2: second vector
            :return: angle
        """
        Lx = np.sqrt(vector1.dot(vector1))
        Ly = np.sqrt(vector2.dot(vector2))
        # print (vector1.dot(vector2)/Lx*Ly)
        return np.arccos(vector1.dot(vector2)/(Lx*Ly))

    def find_angles(self, xz_pos, yz_pos):
        """Finds joints angles
           The robot is redundant, so multiple answers for joints, ignore joint2
            :param xz_pos: list, 5 positions in xz plane [red, green, blue, yellow, target]
            :param yz_pos: list, 5 positions in xz plane
            :return angles
        """
        # centers = [red, green, blue, yellow, target] -- red:[x,y,z] ...
        centers = []
        for i in range(5):
            x = xz_pos[2 * i]
            y = yz_pos[2 * i]
            z = (xz_pos[2 * i + 1] + yz_pos[2 * i + 1])/2
            centers.append(np.array([x, y, z]))
        vector_up = centers[0] - centers[1]
        vector_middle = centers[1] - centers[2]
        vector_down = centers[2] - centers[3]

        joint4 = self.angle_between_vectors(vector_up, vector_middle)
        #joint3 = self.angle_between_vectors(vector_middle, vector_down)
        x_axis = np.array([0, -1, 0])
        proj = np.array([vector_middle[0], vector_middle[1], 0])
        # print(proj)
        proj_yz = np.array([0, vector_middle[1], vector_middle[2]])
        proj_xz = np.array([vector_middle[0], 0, vector_middle[2]])
        joint2 = self.angle_between_vectors(proj_yz, np.array([0, 0, -1]))
        joint3 = self.angle_between_vectors(proj_xz, np.array([0, 0, -1]))
        joint1 = self.angle_between_vectors(proj, x_axis)
        return [joint1, joint2, joint3, joint4]

    def callback(self, xz_pos, yz_pos):
        joint1 = self.find_angles(xz_pos.data, yz_pos.data)[0]
        joint2 = self.find_angles(xz_pos.data, yz_pos.data)[1]
        joint3 = self.find_angles(xz_pos.data, yz_pos.data)[2]
        joint4 = self.find_angles(xz_pos.data, yz_pos.data)[3]
        print('joint1:{:.2f}, joint2:{:.2f}, joint3:{:.2f}, joint4:{:.2f}'.format(joint1, joint2, joint3, joint4))


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