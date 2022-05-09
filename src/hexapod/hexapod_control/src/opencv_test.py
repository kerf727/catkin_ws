#!/usr/bin/env python3
from __future__ import print_function

# import roslib
# roslib.load_manifest('my_package')
# import sys
import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.twist_pub = rospy.Publisher("hexapod/teleop/twist", Twist)
    self.msg = Twist()

    self.state = "N/A"
    self.threshold = 0.30 # distance in meters until it's not safe to get closer to wall

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.callback)

  def callback(self, data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
      depth_array = np.array(cv_image, dtype=np.float32)
      # cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
      p1 = (120, 240)
      p2 = (220, 240)
      p3 = (320, 240)
      p4 = (420, 240)
      p5 = (520, 240)
      
      depth_array = cv2.circle(depth_array, (p1[0], p1[1]), 5, 0, 2)
      depth_array = cv2.circle(depth_array, (p2[0], p2[1]), 5, 0, 2)
      depth_array = cv2.circle(depth_array, (p3[0], p3[1]), 5, 0, 2)
      depth_array = cv2.circle(depth_array, (p4[0], p4[1]), 5, 0, 2)
      depth_array = cv2.circle(depth_array, (p5[0], p5[1]), 5, 0, 2)

    except CvBridgeError as e:
      print(e)
      rospy.loginfo('image msg conversion failed')

    cv2.imshow("Raw Depth Image", depth_array)
    cv2.waitKey(3)

    try:
      d1 = depth_array[p1[1]][p1[0]]
      d2 = depth_array[p2[1]][p2[0]]
      d3 = depth_array[p3[1]][p3[0]]
      d4 = depth_array[p4[1]][p4[0]]
      d5 = depth_array[p5[1]][p5[0]]

      if d1 < self.threshold*1.2 or \
         d2 < self.threshold or \
         d3 < self.threshold or \
         d4 < self.threshold or \
         d5 < self.threshold*1.2:
        self.state = "Turn"
        self.msg.linear.x = 0.0
        self.msg.linear.y = 0.0
        self.msg.angular.x = 1.0
      else:
        self.state = "Go Forward"
        self.msg.linear.x = 0.0
        self.msg.linear.y = 1.0
        self.msg.angular.x = 0.0
      
      self.twist_pub.publish(self.msg)

      print(depth_array[p1[1]][p1[0]],
            depth_array[p2[1]][p2[0]],
            depth_array[p3[1]][p3[0]],
            depth_array[p4[1]][p4[0]],
            depth_array[p5[1]][p5[0]],
            self.state)

    except CvBridgeError as e:
      print(e)

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
