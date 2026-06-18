#!/usr/bin/env python3

import rospy
import numpy as np
import cv2

from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge

class ThermalFeatureExtractor:
    """
    S1-S4 BASELINE (NON-FUZZY)
    
    Extract thermal features only:
    - traversability_score
    - corridor_width  
    - center_path
    
    NO fuzzy logic or cmd_vel generation.
    """

    def __init__(self):

        self.bridge = CvBridge()
        self.traversable_score = 0.0
        self.corridor_width = 0.0
        self.center_path = 0.0
        self.left_trav = 0.0
        self.center_trav = 0.0
        self.right_trav = 0.0
        self.left_score = 0.0
        self.right_score = 0.0
        self.obstacle_mask = None

        # ================================
        # SEMANTIC SUBSCRIBER ONLY
        # (NO LIDAR for feature extraction)
        # ================================

        rospy.Subscriber(
            "/semantic_mask",
            Image,
            self.semantic_callback
        )

        rospy.Subscriber(
            "/thermal_obstacle_mask",
            Image,
            self.obstacle_callback,
            queue_size=1
        )

        # ================================
        # ONLY PUBLISH FEATURES
        # NO cmd_vel or fuzzy_cmd_vel
        # ================================

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

        # Zona kiri/tengah/kanan (lebar = lebar robot, gaya kamera mundur).
        # 1.0 = zona lapang, 0.0 = terhalang. Dipakai APF untuk memutuskan
        # apakah robot muat & ke arah mana menghindar.
        self.zone_pub = rospy.Publisher(
            "/thermal_zones",
            Float32MultiArray,
            queue_size=1
        )

        rospy.loginfo("ThermalFeatureExtractor initialized (S1-S4 BASELINE - NON-FUZZY)")

    # ======================================
    # SEMANTIC FEATURE EXTRACTION
    # ======================================

    def obstacle_callback(self, msg):

        self.obstacle_mask = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='mono8'
        )

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

        obstacle_front = np.zeros_like(front)

        if self.obstacle_mask is not None:
            obstacle = self.obstacle_mask

            if obstacle.shape != frame.shape:
                obstacle = cv2.resize(
                    obstacle,
                    (w, h),
                    interpolation=cv2.INTER_NEAREST
                )

            obstacle_front = obstacle[
                int(h*0.25):int(h*0.99),
                :
            ]

            obstacle_corridor = cv2.bitwise_and(
                obstacle_front,
                mask
            )

            corridor[obstacle_corridor > 100] = 0

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
        obstacle_pixels = np.sum(
            cv2.bitwise_and(
                obstacle_front,
                mask
            ) > 100
        )

        total_pixels = np.sum(mask > 0)

        self.traversable_score = (
            max(0.0, (free_pixels - obstacle_pixels) / total_pixels)
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

        debug[obstacle_front > 100] = (0,0,255)

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
            "THERMAL FEATURE EXTRACTOR",
            debug
        )

        cv2.waitKey(1)

        # ======================================
        # PUBLISH FEATURES ONLY
        # ======================================

        self.trav_pub.publish(Float32(self.traversable_score))
        self.width_pub.publish(Float32(self.corridor_width))
        self.center_pub.publish(Float32(self.center_path))

        # publish 3 zona [kiri, tengah, kanan]
        zone_msg = Float32MultiArray()
        zone_msg.data = [
            float(self.left_trav),
            float(self.center_trav),
            float(self.right_trav)
        ]
        self.zone_pub.publish(zone_msg)

        rospy.loginfo_throttle(
            1.0,
            f"TRAV: {self.traversable_score:.3f} | WIDTH: {self.corridor_width:.1f} | CENTER: {self.center_path:.1f}"
        )


def main():
    rospy.init_node('thermal_feature_extractor', anonymous=True)
    extractor = ThermalFeatureExtractor()
    rospy.spin()


if __name__ == '__main__':
    main()
