#code largely adapted from labs
# !/usr/bin/env python

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
    # initialize a publisher to send images from camera1 to a topic named image_topic1
    self.image_pub1 = rospy.Publisher("image_topic1",Image, queue_size = 1)
    # initialize a publisher to send joints' angular position to a topic called joints_pos
    self.joints_pub1 = rospy.Publisher("joints_pos1",Float64MultiArray, queue_size=10)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw",Image,self.callback1)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()
  
  #detect yellow blob centre
  def detectYellow(self, image):
    yellowBlob = cv2.inRange(image, (25, 0, 0), (50, 255, 255))
    cv2.imshow('window', yellowBlob)
    cv2.waitKey(100000)
    M = cv2.moments(yellowBlob)
    yellowcx = int(M['m10']/M['m00'])
    yellowcy = int(M['m01']/M['m00'])
    yellowCentre = np.array([yellowcx, yellowcy])
    return yellowCentre
  
  #detect blue blob centre
  def detectBlue(self, image):
    blueBlob = cv2.inRange(image, (75, 0, 0), (150, 255, 255))
    cv2.imshow('window', blueBlob)
    cv2.waitKey(100000)
    M = cv2.moments(blueBlob)
    bluecx = int(M['m10']/M['m00'])
    bluecy = int(M['m01']/M['m00'])
    blueCentre = np.array([bluecx, bluecy])
    return blueCentre

  #detect green blob centre
  def detectGreen(self, image):
    greenBlob = cv2.inRange(image, (50, 0, 0), (75, 255, 255))
    cv2.imshow('window', greenBlob)
    cv2.waitKey(100000)
    M = cv2.moments(greenBlob)
    greencx = int(M['m10']/M['m00'])
    greency = int(M['m01']/M['m00'])
    greenCentre = np.array([greencx, greency])
    return greenCentre

  #detect red blob centre
  def detectRed(self, image):
    redBlob = cv2.inRange(image, (0, 100, 100), (25, 255, 255))
    cv2.imshow('window', redBlob)
    cv2.waitKey(100000)
    M = cv2.moments(redBlob)
    redcx = int(M['m10']/M['m00'])
    redcy = int(M['m01']/M['m00'])
    redCentre = np.array([redcx, redcy])
    return redCentre

  #convert from pixel to meter
  def pix2m(self, image):
    dist = np.sqrt(np.sum((self.detectYellow(image) - self.detectBlue(image)) ** 2))
    return 2 / dist #link is 2 meter according to specification

  #get joint angles
  def detectJointAngles(self, image):
    img1 = self.pix2m(image)
    yellowCentre = self.detectYellow(img1)
    blueCentre = self.detectBlue(img1)
    greenCentre = self.detectGreen(img1)
    redCentre = self.detectRed(img1)
    jointAngle1 = np.arctan2(yellowCentre[0] - blueCentre[0], yellowCentre[1] - blueCentre[1])
    jointAngle2 = np.arctan2(blueCentre[0] - greenCentre[0], blueCentre[1] - greenCentre[0]) - jointAngle1
    jointAngle3 = np.arctan2(greenCentre[0] - redCentre[0], greenCentre[1] - redCentre[1])  - jointAngle1 - jointAngle2
    return np.array([jointAngle1, jointAngle2,jointAngle3])



  # Recieve data from camera 1, process it, and publish
  def callback1(self,data):
    # Recieve the image
    try:
      self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', cv_image)
    
    #image processing
    #im1=cv2.imshow('window1', self.cv_image1)
    #cv2.waitKey(1)

    hsvImage1 = cv2.cvtColor(self.cv_image1, cv2.COLOR_BGR2HSV) # convert image to HSV
    jointAngles = self.detectJointAngles(hsvImage1)

    self.joints = Float64MultiArray()
    self.joints.data = jointAngles

    # Publish the results
    try: 
      self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
      self.joints_pub1.publish(self.joints)
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


