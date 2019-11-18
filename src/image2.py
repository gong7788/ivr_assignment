#!/usr/bin/env python

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
# XZ camera
  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # initialize a publisher to send images from camera2 to a topic named image_topic2
    self.image_pub2 = rospy.Publisher("image_topic2",Image, queue_size = 1)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw",Image,self.callback2)
    # initialize target position
    self.target_pub = rospy.Publisher("/camera2/target_position", Float64MultiArray, queue_size=10)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()
    self.target_pre = np.array([0, 0])

  def detect_red(self, image):
    """Finds the position of red joint(end-effect)

    :param image: matrix, width*height*3
        The image captured by camera2
    :return: array, [x, y]
        The position of red joint in pixel (Top-left:[0 , 0], Right-down:[width, height])
    """
    mask = cv2.inRange(image, (0, 0, 100), (100, 100, 255))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel=np.ones((5, 5), dtype=np.uint8))
    # cv2.imshow('red2', mask)
    # cv2.waitKey(2)
    M = cv2.moments(mask)
    if M['m00'] == 0:
      # If not detect red, return green position
      return self.detect_green(image)
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    return np.array([cx, cy])

  def detect_green(self, image):
    """Finds the position of green joint(end-effect)

        :param image: matrix, width*height*3
            The image captured by camera2
        :return: array, [x, y]
            The position of red joint in pixel (Top-left:[0 , 0], Right-down:[width, height])
    """
    mask = cv2.inRange(image, (0, 100, 0), (100, 255, 100))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel=np.ones((2, 2), dtype=np.uint8))
    # cv2.imshow('green2', mask)
    # cv2.waitKey(2)
    M = cv2.moments(mask)
    if M['m00'] == 0:
      # If not detect green, return blue position
      return self.detect_blue(image)
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    # print(cx,cy)
    return np.array([cx, cy])

  def detect_blue(self, image):
    """Finds the position of blue joint(end-effect)

        :param image: matrix, width*height*3
            The image captured by camera2
        :return: array, [x, y]
            The position of red joint in pixel (Top-left:[0 , 0], Right-down:[width, height])
    """
    mask = cv2.inRange(image, (100, 0, 0), (255, 100, 100))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel=np.ones((5, 5), dtype=np.uint8))
    M = cv2.moments(mask)
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    return np.array([cx, cy])

  def detect_yellow(self, image):
    """Finds the position of yellow joint(end-effect)

        :param image: matrix, width*height*3
            The image captured by camera2
        :return: array, [x, y]
            The position of red joint in pixel (Top-left:[0 , 0], Right-down:[width, height])
    """
    mask = cv2.inRange(image, (0, 100, 100), (100, 255, 139))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel=np.ones((5, 5), dtype=np.uint8))
    # cv2.imshow('window 3', mask)
    # cv2.waitKey(2)
    M = cv2.moments(mask)
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    return np.array([cx, cy])

  def detect_target(self, image):
    """Finds the position of orange sphere

    :param image:  matrix, width*height*3
        The image captured by camera2
    :return: array, [x, y]
        The position of orange sphere in pixel (Top-left:[0 , 0], Right-down:[width, height])
    """
    image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(image, (11, 43, 46), (25, 255, 255))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel=np.ones((5, 5), dtype=np.uint8))
    # cv2.imshow('target2', mask)
    # cv2.waitKey(2)
    M = cv2.moments(mask)
    if M['m00'] == 0:
      #TODO target after red or green joint
      return self.target_pre
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    self.target_pre = np.array([cx,cy])
    return np.array([cx, cy])


  # Recieve data, process it, and publish
  def callback2(self,data):
    # Recieve the image
    try:
      self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', cv_image)
    # im2=cv2.imshow('XZ, Camera2', self.cv_image2)
    # cv2.waitKey(1)

    # XZ_positions = [red, green, blue, yellow, target]
    red_pos = self.detect_red(self.cv_image2)
    green_pos = self.detect_green(self.cv_image2)
    blue_pos = self.detect_blue(self.cv_image2)
    yellow_pos = self.detect_yellow(self.cv_image2)
    target_position = self.detect_target(self.cv_image2)

    XZ_positions = np.concatenate((red_pos, green_pos, blue_pos, yellow_pos, target_position), axis=0)

    positions = Float64MultiArray()
    positions.data = XZ_positions

    # print('red', red_pos)
    # print('green', green_pos)
    # print('blue', blue_pos)
    # print('yellow', yellow_pos)

    # Publish the results
    try:
      self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
      self.target_pub.publish(positions)
    except CvBridgeError as e:
      print(e)

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


