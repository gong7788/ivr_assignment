import numpy as np
from numpy import sin
from numpy import cos

class Solution:

    def __init__(self):
        self.time_previous = 0

    def calculate_jacobian(self, angles):
        q1 = angles[0]
        q2 = angles[1]
        q3 = angles[2]
        q4 = angles[3]
        jacobian = np.array([[3*cos(q1)*cos(q3)*sin(q2)-3*sin(q1)*sin(q3)+2*cos(q4)(cos(q1)*cos(q3)*sin(q2)-sin(q1)*sin(q3))+2*cos(q1)*cos(q2)*sin(q4),
                              3*cos(q2)*cos(q3)*sin(q1)+2*cos(q2)*cos(q3)*cos(q4)*sin(q1)-2*sin(q1)*sin(q2)*sin(q4),
                              3*cos(q1)*cos(q3)-3*sin(q1)*sin(q2)*sin(q3)+2*cos(q4)(cos(q1)*cos(q3)-sin(q1)*sin(q2)*sin(q3)),
                              2*cos(q2)*cos(q4)*sin(q1)-2(cos(q3)*sin(q1)*sin(q2)+cos(q1)*sin(q3))*sin(q4)],
                             [3*cos(q3)*sin(q1)*sin(q2)+3*cos(q1)*sin(q3)+2*cos(q4)(cos(q3)*sin(q1)*sin(q2)+cos(q1)*sin(q3))+2*cos(q2)*sin(q1)*sin(q4),
                              -3*cos(q1)*cos(q2)*cos(q3)-2*cos(q1)*cos(q2)*cos(q3)*cos(q4)+2*cos(q1)*sin(q2)*sin(q4),
                              3*cos(q3)*sin(q1)+3*cos(q1)*sin(q2)*sin(q3)+2*cos(q4)(cos(q3)*sin(q1)+cos(q1)*sin(q2)*sin(q3)),
                              -2*cos(q1)*cos(q2)*cos(q4)-2(-cos(q1)*cos(q3)*sin(q2)+sin(q1)*sin(q3))*sin(q4)],
                             [0,
                              -3*cos(q3)*sin(q2)-2*cos(q3)*cos(q4)*sin(q2)-2*cos(q2)*sin(q4),
                              -3*cos(q2)*sin(q3)-2*cos(q2)*cos(q4)*sin(q3),
                              -2*cos(q4)*sin(q2)-2*cos(q2)*cos(q3)*sin(q4)]])
        return jacobian

    def control_closed(self, angles):
        """Finds

            :param angles:   list of joints(initial) [q1, q2, q3, q4]
            :return          joints angles [q1, q2, q3, q4]
        """
        # P gains
        K_p = np.array([[1, 0, 0],
                        [0, 1, 0],
                        [0, 0, 1]])
        # D gains
        K_d = np.array([[0.1, 0, 0],
                        [0, 0.1, 0],
                        [0, 0, 0.1]])

        cur_time = rospy.get_time()
        dt = cur_time - self.time_previous
        self.time_previous = cur_time
        # robot end-effector position
        pos_end = self.target_coordinates(red, yellow, blue)
        # target position
        pos_target = self.target_coordinates(target, yellow, blue)
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


