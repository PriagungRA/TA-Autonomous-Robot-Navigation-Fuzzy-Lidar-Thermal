#!/usr/bin/env python3

import rospy
import numpy as np
import cv2

from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

from cv_bridge import CvBridge

class FuzzyNavigation:

    def __init__(self):

        self.bridge = CvBridge()

        self.front_distance = 10.0

        self.left_distance = 10.0

        self.right_distance = 10.0

        self.traversable_score = 0.0

        rospy.Subscriber(
            "/scan",
            LaserScan,
            self.scan_callback
        )

        rospy.Subscriber(
            "/semantic_mask",
            Image,
            self.semantic_callback
        )

        self.cmd_pub = rospy.Publisher(
            "/fuzzy_cmd_vel",
            Twist,
            queue_size=1
        )

        rospy.Timer(
            rospy.Duration(0.1),
            self.control_loop
        )

    # ======================================
    # LIDAR
    # ======================================

    def scan_callback(self, msg):

        ranges = np.array(msg.ranges)

        ranges[np.isinf(ranges)] = 10.0

        self.front_distance = np.min(ranges[170:190])

        self.left_distance = np.min(ranges[260:320])

        self.right_distance = np.min(ranges[40:100])

    # ======================================
    # SEMANTIC
    # ======================================

    def semantic_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='mono8'
        )

        h, w = frame.shape

        # ======================================
        # POLYGON ROI
        # ======================================

        mask = np.zeros_like(frame)

        pts = np.array([

            [220,470],
            [420,470],
            [520,260],
            [120,260]

        ])

        cv2.fillPoly(
            mask,
            [pts],
            255
        )

        roi = cv2.bitwise_and(
            frame,
            mask
        )

        # ======================================
        # DEBUG VIEW
        # ======================================

        debug = cv2.cvtColor(
            frame,
            cv2.COLOR_GRAY2BGR
        )

        cv2.polylines(
            debug,
            [pts],
            True,
            (255,255,255),
            2
        )

        cv2.imshow("ROI DEBUG", debug)

        cv2.imshow("ROI", roi)

        cv2.waitKey(1)

        # ======================================
        # TRAVERSABILITY SCORE
        # ======================================

        free_pixels = np.sum(roi > 100)

        total_pixels = np.sum(mask > 0)

        self.traversable_score = (
            free_pixels / total_pixels
        )

        print("TRAV SCORE :", self.traversable_score)
        print("TRAV SCORE :", self.traversable_score)

    # ======================================
    # FUZZY CONTROL
    # ======================================

    def control_loop(self, event):

        cmd = Twist()

        print("================================")
        print("FRONT :", self.front_distance)
        print("LEFT  :", self.left_distance)
        print("RIGHT :", self.right_distance)
        print("TRAV  :", self.traversable_score)

        # ======================================
        # RULE 1
        # obstacle dekat tapi jalan masih ada
        # ======================================

        if self.front_distance < 0.8:

            if self.traversable_score > 0.25:

                print("RULE : SLOW FORWARD")

                cmd.linear.x = 0.05

                # pilih arah paling lega

                if self.left_distance > self.right_distance:

                    cmd.angular.z = 0.2

                else:

                    cmd.angular.z = -0.2

            else:

                print("RULE : ROTATE / REPLAN")

                cmd.linear.x = 0.0

                if self.left_distance > self.right_distance:

                    cmd.angular.z = 0.5

                else:

                    cmd.angular.z = -0.5

        # ======================================
        # RULE 2
        # aman
        # ======================================

        else:

            print("RULE : SAFE")

            cmd.linear.x = 0.0

            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

if __name__ == "__main__":

    rospy.init_node("fuzzy_navigation")

    FuzzyNavigation()

    rospy.spin()