#!/usr/bin/env python3

import cmd
import rospy
import numpy as np
import cv2

from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from cv_bridge import CvBridge

class FuzzyNavigation:

    def __init__(self):

        self.bridge = CvBridge()
        self.front_distance = 10.0
        self.left_distance = 10.0
        self.right_distance = 10.0
        self.traversable_score = 0.0
        self.left_score = 0.0
        self.right_score = 0.0
        self.corridor_width = 0.0
        self.center_path = 0.0
        self.left_trav = 0.0
        self.center_trav = 0.0
        self.right_trav = 0.0

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

        self.trav_pub = rospy.Publisher(
            "/traversability_score",
            Float32,
            queue_size=1
        )

        self.width_pub = rospy.Publisher(
            "/corridor_width",
            Float32,
            queue_size=1
        )

        self.center_pub = rospy.Publisher(
            "/center_path",
            Float32,
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
            desired_encoding='bgr8'
        )

        frame = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2GRAY
        )

        h, w = frame.shape

        # ======================================
        # FRONT AREA
        # ======================================

        front = frame[
            int(h*0.25):int(h*0.99),
            :
        ]

        fh, fw = front.shape

        # ======================================
        # TRAPEZOID ROI
        # ======================================

        mask = np.zeros_like(front)

        pts = np.array([

            [90,  fh],
            [550, fh],
            [430, 140],
            [210, 140]

        ])

        cv2.fillPoly(
            mask,
            [pts],
            255
        )

        corridor = cv2.bitwise_and(
            front,
            mask
        )

        # ======================================
        # LEFT CENTER RIGHT ROI
        # ======================================

        left_mask = np.zeros_like(front)
        left_pts = np.array([
            [212,143],
            [285,143],
            [244,356],
            [90,356]
        ])
        cv2.fillPoly(
            left_mask,
            [left_pts],
            255
        )

        center_mask = np.zeros_like(front)
        center_pts = np.array([
            [285,143],
            [359,143],
            [398,356],
            [244,356]
        ])
        cv2.fillPoly(
            center_mask,
            [center_pts],
            255
        )

        right_mask = np.zeros_like(front)
        right_pts = np.array([
            [359,143],
            [432,143],
            [550,356],
            [398,356]
        ])
        cv2.fillPoly(
            right_mask,
            [right_pts],
            255
        )       

        left_roi = cv2.bitwise_and(
            corridor,
            left_mask
        )

        center_roi = cv2.bitwise_and(
            corridor,
            center_mask
        )

        right_roi = cv2.bitwise_and(
            corridor,
            right_mask
        )

        self.left_trav = (
            np.sum(left_roi > 100)
            /
            np.sum(left_mask > 0)
        )

        self.center_trav = (
            np.sum(center_roi > 100)
            /
            np.sum(center_mask > 0)
        )

        self.right_trav = (
            np.sum(right_roi > 100)
            /
            np.sum(right_mask > 0)
        )

        # ======================================
        # CORRIDOR WIDTH ESTIMATION
        # ======================================

        row = corridor[int(fh*0.75), :]

        print("ROW MIN :", np.min(row))
        print("ROW MAX :", np.max(row))
        print("ROW MEAN:", np.mean(row))

        free_idx = np.where(row > 100)[0]
        if len(free_idx) > 0:
            corridor_width = (
                np.max(free_idx)
                -
                np.min(free_idx)
            )
            center_path = np.mean(free_idx)
        else:
            corridor_width = 0
            center_path = fw / 2
        self.corridor_width = corridor_width
        self.center_path = center_path

        # ======================================
        # SCORE
        # ======================================

        free_pixels = np.sum(corridor > 100)

        total_pixels = np.sum(mask > 0)

        self.traversable_score = (
            free_pixels / total_pixels
        )

        # ======================================
        # LEFT RIGHT ANALYSIS
        # ======================================

        left_area = corridor[:, 0:int(fw*0.5)]

        right_area = corridor[:, int(fw*0.5):fw]

        self.left_score = (
            np.sum(left_area > 100)
            / left_area.size
        )

        self.right_score = (
            np.sum(right_area > 100)
            / right_area.size
        )

        # ======================================
        # DEBUG VIEW
        # ======================================

        debug = cv2.cvtColor(
            front,
            cv2.COLOR_GRAY2BGR
        )

        overlay = debug.copy()

        cv2.fillPoly(
            overlay,
            [pts],
            (255,255,0)
        )

        alpha = 0.25

        debug = cv2.addWeighted(
            overlay,
            alpha,
            debug,
            1-alpha,
            0
        )

        cv2.polylines(
            debug,
            [left_pts],
            True,
            (0,255,0),
            2
        )

        cv2.polylines(
            debug,
            [center_pts],
            True,
            (255,0,0),
            2
        )

        cv2.polylines(
            debug,
            [right_pts],
            True,
            (0,0,255),
            2
        )

        cv2.polylines(
            debug,
            [pts],
            True,
            (255,255,255),
            3
        )

        cv2.circle(
            debug,
            (
                int(self.center_path),
                int(fh*0.75)
            ),
            8,
            (0,0,255),
            -1
        )

        cv2.imshow(
            "TRAPEZOID ROI",
            debug
        )

        cv2.imshow(
            "CORRIDOR",
            corridor
        )

        cv2.waitKey(1)

        # ======================================
        # PUBLISH & PRINT
        # ======================================

        self.trav_pub.publish(Float32(self.traversable_score))
        self.width_pub.publish(Float32(self.corridor_width))
        self.center_pub.publish(Float32(self.center_path))

        print("TRAV SCORE :", self.traversable_score)
        print("CORRIDOR WIDTH:", self.corridor_width)
        print("CENTER PATH:", self.center_path)
        print("LEFT SCORE :", self.left_score)
        print("RIGHT SCORE:", self.right_score)
        print("LEFT TRAV  :", self.left_trav)
        print("CENTER TRAV:", self.center_trav)
        print("RIGHT TRAV :", self.right_trav)

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
        # THERMAL AVOIDANCE
        # ======================================

        if self.center_trav < 0.85:

            print("RULE : THERMAL AVOID")

            cmd.linear.x = 0.15

            if self.left_trav > self.right_trav:

                print("GO LEFT")

                cmd.angular.z = 0.4

            else:

                print("GO RIGHT")

                cmd.angular.z = -0.4

        elif self.center_trav < 0.55:

            print("CENTER BLOCKED")

            cmd.linear.x = 0.15

            if self.left_trav > self.right_trav:
                cmd.angular.z = 0.45
            else:
                cmd.angular.z = -0.45

        # ======================================
        # RULE 1
        # obstacle dekat tapi jalan masih ada
        # ======================================

        elif self.front_distance < 0.4:

            if (self.traversable_score > 0.25 and self.corridor_width > 220):

                print("RULE : SLOW FORWARD")

                cmd.linear.x = 0.05

                # pilih arah paling lega

                if self.left_score > self.right_score:

                    cmd.angular.z = 0.2

                else:

                    cmd.angular.z = -0.2

            else:

                print("RULE : ROTATE / REPLAN")

                cmd.linear.x = 0.0

                if self.left_score > self.right_score:

                    cmd.angular.z = 0.5

                else:

                    cmd.angular.z = -0.5

        elif self.front_distance < 1.0:

            print("RULE : SLOW AVOID")

            cmd.linear.x = 0.10

            if self.left_score > self.right_score:
                cmd.angular.z = 0.3
            else:
                cmd.angular.z = -0.3

        # ======================================
        # RULE 2
        # aman
        # ======================================

        else:

            print("RULE : SAFE")

            if self.corridor_width > 300:
                cmd.linear.x = 0.35

            elif self.corridor_width > 220:
                cmd.linear.x = 0.25

            else:
                cmd.linear.x = 0.15
            # ======================================
            # CENTERING STEERING
            # ======================================

            image_center = 320
            error = (
                self.center_path
                -
                image_center
            )
            if abs(error) < 25:
                cmd.angular.z = 0.0
            else:
                cmd.angular.z = np.clip(
                    -error * 0.0025,
                    -0.6,
                    0.6
                )

        self.cmd_pub.publish(cmd)

if __name__ == "__main__":

    rospy.init_node("fuzzy_navigation")

    FuzzyNavigation()

    rospy.spin()