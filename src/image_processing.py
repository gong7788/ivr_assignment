#!/usr/bin/env python

import image2
import image1

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError

class image_converter:
    # Defines publisher and subscriber
    def __init__(self):
        # initialize the node named image_processing
        rospy.init_node('image_processing', anonymous=True)

        self.subscriber_name = rospy.Subscriber("/camera2/target_position", Float64MultiArray, self.callback)
        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()

    #def callback(self):

    # Calculate the centres for the joints
    def calculate_centres(self):
        red_centre = image1.detect_red.append(image2.detect_red[0], 1)
        green_centre = image1.detect_green.append(image2.detect_green[0], 1)
        blue_centre = image1.detect_blue.append(image2.detect_blue[0], 1)
        yellow_centre = image1.detect_yellow.append(image2.detect_yellow[0], 1)
        centres = [red_centre, green_centre, blue_centre, yellow_centre]
        return centres

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