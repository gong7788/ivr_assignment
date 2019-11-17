#!/usr/bin/env python

import roslib
from scipy.optimize import leastsq
from scipy.optimize import least_squares
import math
import sys
import rospy
import cv2
import numpy as np
from numpy import sin
from numpy import cos
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
        self.time_previous = rospy.get_time()
        self.error = np.array([0, 0, 0])
        self.error_d = np.array([0, 0, 0])

        self.target_pub = rospy.Publisher('/target/detected', Float64MultiArray, queue_size=10)

        self.joints_pub = rospy.Publisher("joints_pos", Float64MultiArray, queue_size=10)
        self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
        self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
        self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
        self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)

        self.XZ_position = message_filters.Subscriber("/camera2/target_position", Float64MultiArray)
        self.YZ_position = message_filters.Subscriber("/camera1/target_position", Float64MultiArray)
        self.sync = message_filters.ApproximateTimeSynchronizer([self.XZ_position, self.YZ_position], 10, 0.1,
                                                                allow_headerless=True)
        self.sync.registerCallback(self.callback)


    def fun_trans(self, x, data):
        return ((3 * np.cos(x[2]) * np.sin(x[0]) * np.sin(x[1]) + 3 * np.cos(x[0]) * np.sin(x[2]) - data[0]),
                (-3 * np.cos(x[0]) * np.cos(x[2]) * np.sin(x[1]) + 3 * np.sin(x[0]) * np.sin(x[2]) - data[1]),
                (2 + 3 * np.cos(x[1]) * np.cos(x[2]) - data[2]),
                (2 + 3 * np.cos(x[1]) * np.cos(x[2]) + 2 * np.cos(x[1]) * np.cos(x[2]) * np.cos(x[3]) - 2 * np.sin(
                    x[1]) * np.sin(x[3]) - data[3]))

    def error(self, theta, true):
        temp = self.fun_trans(theta)
        estimated = np.array([temp[0][0], temp[1][0], temp[2][0], temp[3][0]])
        return np.sum((estimated - true) ** 2)

    def find_angles(self, end_effector, green):
        """Finds joints angles
            First using end effector coordinate to calculate angle1, angle2 and angle3, then using those
            angles and green coordinate to calculate angle4
            :param end_effector:      coordinates of end effector [x, y, z]
            :param green:             coordinates of green [x, y, z]
            :return angles            four angles [q1, q2, q3, q4]
        """
        true = [green[0], green[1], green[2], end_effector[2]]
        np.set_printoptions(suppress=True)
        # res_1 = least_squares(self.error, x0=[0, 0, 0, 0],
        #                       bounds=([-math.pi, -math.pi / 2, -math.pi / 2, -math.pi / 2], [math.pi, math.pi / 2, math.pi / 2, math.pi / 2]),
        #                       args = true)
        res_1 = leastsq(self.fun_trans, (0, 0, 0, 0), args=true)
        # print(res_1[0])
        return res_1[0]

    def target_coordinates(self, target, base):
        """Transforms pixel coordinates into meters

            :param target: [x,y,z] target coordinates in pixel
            :param base:   [x,y,z] base(yellow joint) coordinates in pixel
            :return        [x,y,z] coordinates in meters
        """
        target_base = 0.0381 * (target - base)
        target_base[2] = -target_base[2]
        return target_base

    def FK(self, input_theta):
        """ Using Forward Kinematics to find end effector position

            :param input_theta:     the angles calculated by the cameras
            :return                 red(end-effector) position and green position   [red, green]
        """
        theta = [1.57 + input_theta[0], 1.57 + input_theta[1], input_theta[2], -input_theta[3]]
        a = [0, 0, 3, 2]
        alpha = [1.57, 1.57, 1.57, 0]
        d = [2, 0, 0, 0]

        Transform = []
        for i in range(4):
            T_theta = np.array([[np.cos(theta[i]), -np.sin(theta[i]), 0, 0],
                                [np.sin(theta[i]), np.cos(theta[i]), 0, 0],
                                [0, 0, 1, d[i]],
                                [0, 0, 0, 1]])

            T_alpha = np.array([[1, 0, 0, a[i]],
                                [0, np.cos(alpha[i]), -np.sin(alpha[i]), 0],
                                [0, np.sin(alpha[i]), np.cos(alpha[i]), 0],
                                [0, 0, 0, 1]])

            Transform.append(T_theta.dot(T_alpha))

        red_co = Transform[0].dot(Transform[1]).dot(Transform[2]).dot(Transform[3])[0:3, -1]
        green_co = Transform[0].dot(Transform[1]).dot(Transform[2])[0:3, -1]
        # blue_co = Transform[0][0:3, -1]

        return [red_co, green_co]

    def calculate_jacobian(self, angles):
        # calculate jacobian matrix
        q1 = angles[0]
        q2 = angles[1]
        q3 = angles[2]
        q4 = angles[3]
        jacobian = np.array([[3*cos(q1)*cos(q3)*sin(q2)-3*sin(q1)*sin(q3)+2*cos(q4)*(cos(q1)*cos(q3)*sin(q2)-sin(q1)*sin(q3))+2*cos(q1)*cos(q2)*sin(q4),
                              3*cos(q2)*cos(q3)*sin(q1)+2*cos(q2)*cos(q3)*cos(q4)*sin(q1)-2*sin(q1)*sin(q2)*sin(q4),
                              3*cos(q1)*cos(q3)-3*sin(q1)*sin(q2)*sin(q3)+2*cos(q4)*(cos(q1)*cos(q3)-sin(q1)*sin(q2)*sin(q3)),
                              2*cos(q2)*cos(q4)*sin(q1)-2*(cos(q3)*sin(q1)*sin(q2)+cos(q1)*sin(q3))*sin(q4)],
                             [3*cos(q3)*sin(q1)*sin(q2)+3*cos(q1)*sin(q3)+2*cos(q4)*(cos(q3)*sin(q1)*sin(q2)+cos(q1)*sin(q3))+2*cos(q2)*sin(q1)*sin(q4),
                              -3*cos(q1)*cos(q2)*cos(q3)-2*cos(q1)*cos(q2)*cos(q3)*cos(q4)+2*cos(q1)*sin(q2)*sin(q4),
                              3*cos(q3)*sin(q1)+3*cos(q1)*sin(q2)*sin(q3)+2*cos(q4)*(cos(q3)*sin(q1)+cos(q1)*sin(q2)*sin(q3)),
                              -2*cos(q1)*cos(q2)*cos(q4)-2*(-cos(q1)*cos(q3)*sin(q2)+sin(q1)*sin(q3))*sin(q4)],
                             [0,
                              -3*cos(q3)*sin(q2)-2*cos(q3)*cos(q4)*sin(q2)-2*cos(q2)*sin(q4),
                              -3*cos(q2)*sin(q3)-2*cos(q2)*cos(q4)*sin(q3),
                              -2*cos(q4)*sin(q2)-2*cos(q2)*cos(q3)*sin(q4)]])
        return jacobian

    def control_closed(self, angles, target, red, yellow):
        """ Controling robot using PD control

            :param angles:   list of joints(initial) [q1, q2, q3, q4]
            :return          joints angles [q1, q2, q3, q4]
        """
        # P gains
        K_p = np.array([[10, 0, 0],
                        [0, 10, 0],
                        [0, 0, 10]])
        # D gains
        K_d = np.array([[0.5, 0, 0],
                        [0, 0.5, 0],
                        [0, 0, 0.5]])

        cur_time = rospy.get_time()
        dt = cur_time - self.time_previous
        self.time_previous = cur_time
        # robot end-effector position
        pos_end = self.target_coordinates(red, yellow)
        # target position
        pos_target = self.target_coordinates(target, yellow)
        # estimate derivative of error
        self.error_d = ((pos_target - pos_end) - self.error)/dt
        # estimate error
        self.error = pos_target - pos_end

        J_inv = np.linalg.pinv(self.calculate_jacobian(angles))  # calculate the psudeo inverse of Jacobian
        # angular velocity of joints
        dq_d = np.dot(J_inv, (np.dot(K_d, self.error_d.T) + np.dot(K_p, self.error.T)))
        # angular position of joints
        q_d = angles + (dt * dq_d)
        return q_d

    def callback(self, xz_pos, yz_pos):
        centers = []
        for i in range(5):
            x = xz_pos.data[2 * i]
            y = yz_pos.data[2 * i]
            z = (xz_pos.data[2 * i + 1] + yz_pos.data[2 * i + 1]) / 2
            centers.append(np.array([x, y, z]))

        # joint1 = self.find_angles(centers)[0]
        # joint2 = self.find_angles(centers)[1]
        # joint3 = self.find_angles(centers)[2]
        # joint4 = self.find_angles(centers)[3]
        # print('joint1:{:.2f}, joint2:{:.2f}, joint3:{:.2f}, joint4:{:.2f}'.format(joint1, joint2, joint3, joint4))
        red = centers[0]
        yellow = centers[-2]
        target = centers[-1]

        target_pub = Float64MultiArray()
        target_pub.data = self.target_coordinates(target, yellow)

        np.set_printoptions(suppress=True)

        # ---------------Test 1.1, find angles give image------------------------
        #TODO: Testing 10 angles

        true_end = np.array(self.target_coordinates(centers[0], yellow))
        green = np.array(self.target_coordinates(centers[1], yellow))
        estimate_angles = self.find_angles(true_end, green)
        print("Estimated angles: ", estimate_angles)
        # print(true_end)
        # print("Estimated coordinates: ", self.FK(estimate_angles)[0])
        # print("Error: ", np.sqrt(np.sum((self.FK(estimate_angles)[0] - true_end) ** 2)))

        # ---------------------Test FK---------------------------------------------
        # input_theta = [0, 0, 1, 0]
        # FK_end = np.array(self.FK(input_theta)[0])
        # print("True end effector: ", true_end)
        # print("Green", green)
        # print("FK end effector: ", FK_end)
        # print("Euclidean distance error: ", np.sqrt( np.sum((FK_end - true_end)**2)))
        # print("Green: ", self.target_coordinates(centers[1], yellow, blue))
        # print("Blue: ", self.target_coordinates(centers[2], yellow, blue))

        # ---------------------------Test PD control-----------------------------------
        #TODO: Test np.array([rospy.get_time()]) and rospy.get_time()
        #TODO: Test PD

        q_d = self.control_closed(estimate_angles, target, red, yellow)
        print ('target', target)
        # --------------------------Set joints-----------------------------------------
        # control the robot joints
        self.joint1 = Float64()
        self.joint1.data = q_d[0]
        self.joint2 = Float64()
        self.joint2.data = q_d[1]
        self.joint3 = Float64()
        self.joint3.data = q_d[2]
        self.joint4 = Float64()
        self.joint4.data = q_d[3]
        # --------------------------Publish the results--------------------------------
        try:
            self.robot_joint1_pub.publish(self.joint1)
            self.robot_joint2_pub.publish(self.joint2)
            self.robot_joint3_pub.publish(self.joint3)
            self.robot_joint4_pub.publish(self.joint4)
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
