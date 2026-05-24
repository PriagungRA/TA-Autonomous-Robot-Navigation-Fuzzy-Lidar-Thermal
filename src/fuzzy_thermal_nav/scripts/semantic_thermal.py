#!/usr/bin/env python3

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class SemanticThermal:

    def __init__(self):

        self.bridge = CvBridge()

        rospy.Subscriber(
            "/thermal/thermal_camera/image_raw",
            Image,
            self.callback,
            queue_size=1
        )

        self.pub = rospy.Publisher(
            "/semantic_mask",
            Image,
            queue_size=1
        )

    # ======================================
    # CALLBACK
    # ======================================

    def callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='mono8'
        )

        frame = cv2.rotate(
            frame,
            cv2.ROTATE_90_CLOCKWISE
        )

        h, w = frame.shape

        frame = frame[
            int(h*0.33):int(h*0.66),
            int(w*0.7):int(w*0.99)
            :
        ]

        # blur ringan
        blur = cv2.GaussianBlur(
            frame,
            (1,1),
            0
        )

        # edge detection
        edges = cv2.Canny(
            blur,
            80,
            150
        )

        # brightness mask
        _, bright = cv2.threshold(
            blur,
            40,
            255,
            cv2.THRESH_BINARY
        )

        # combine
        semantic = cv2.bitwise_and(
            bright,
            255 - edges
        )

        # # smoothing
        # semantic = cv2.GaussianBlur(
        #     semantic,
        #     (7,7),
        #     0
        # )

        ros_img = self.bridge.cv2_to_imgmsg(
            semantic,
            encoding='mono8'
        )

        self.pub.publish(ros_img)

if __name__ == "__main__":

    rospy.init_node("semantic_thermal")

    SemanticThermal()

    rospy.spin()