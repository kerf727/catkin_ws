#!/usr/bin/env python3
from __future__ import print_function

# import roslib
# roslib.load_manifest('my_package')
# import sys
import rospy
import cv2
import numpy as np
# from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    # self.image_pub = rospy.Publisher("image_topic_2",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.callback)

  def callback(self, data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
      depth_array = np.array(cv_image, dtype=np.float32)
      # cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
      # checkPoints(depth_array)
      p1 = (120, 240)
      p2 = (220, 240)
      p3 = (320, 240)
      p4 = (420, 240)
      p5 = (520, 240)
      print(depth_array[p1[1]][p1[0]],
            depth_array[p2[1]][p2[0]],
            depth_array[p3[1]][p3[0]],
            depth_array[p4[1]][p4[0]],
            depth_array[p5[1]][p5[0]])
      depth_array = cv2.circle(depth_array, (p1[0], p1[1]), 5, 0, 2)
      depth_array = cv2.circle(depth_array, (p2[0], p2[1]), 5, 0, 2)
      depth_array = cv2.circle(depth_array, (p3[0], p3[1]), 5, 0, 2)
      depth_array = cv2.circle(depth_array, (p4[0], p4[1]), 5, 0, 2)
      depth_array = cv2.circle(depth_array, (p5[0], p5[1]), 5, 0, 2)

      threshold = 0.15 # meters until it's not safe to get closer to wall
      # depth_array = cv2.line(depth_array, (), (), 0, 5)
    except CvBridgeError as e:
      print(e)
      rospy.loginfo('image msg conversion failed')

    # (rows,cols,channels) = cv_image.shape
    # if cols > 60 and rows > 60 :
    #   cv2.circle(cv_image, (50,50), 10, 255)

    cv2.imshow("Raw Depth Image", depth_array)
    cv2.waitKey(3)

    # try:
    #   self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    # except CvBridgeError as e:
    #   print(e)

  # def checkPoints(self, depth_array):
  #   p1 = (100, 240)
  #   p2 = (200, 240)
  #   p3 = (300, 240)
  #   p4 = (400, 240)
  #   p5 = (500, 240)
  #   print(depth_array[p1[0]][p1[1]],
  #         depth_array[p2[0]][p2[1]],
  #         depth_array[p3[0]][p3[1]],
  #         depth_array[p4[0]][p4[1]],
  #         depth_array[p5[0]][p5[1]])

def main():
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  rospy.loginfo('image_converter node started')
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
  main()
