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

        self.obstacle_pub = rospy.Publisher(
            "/thermal_obstacle_mask",
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
            int(h*0.34):int(h*0.64),
            int(w*0.7):int(w*0.99)
        ]

        frame = cv2.resize(
            frame,
            (640,480),
            interpolation=cv2.INTER_NEAREST
        )

        rospy.loginfo_throttle(
            1.0,
            "RAW MIN: %d MAX: %d MEAN: %.2f",
            np.min(frame),
            np.max(frame),
            np.mean(frame)
        )

        # blur ringan
        blur = cv2.GaussianBlur(
            frame,
            (3,3),
            0
        )

        # sharpen edge
        sharp = cv2.addWeighted(
            frame,
            1.5,
            blur,
            -0.5,
            0
        )

        rospy.loginfo_throttle(
            1.0,
            "SHARP MIN: %d MAX: %d MEAN: %.2f",
            np.min(sharp),
            np.max(sharp),
            np.mean(sharp)
        )

        # ======================================
        # SIMPLE THERMAL SEGMENTATION
        # ======================================

        # Jalan pada thermal terlihat abu-abu sedang. Obstacle thermal
        # yang lebih panas/putih sengaja dikeluarkan dari range ini.
        road_min = 45
        road_max = 72

        semantic = cv2.inRange(
            sharp,
            road_min,
            road_max
        )

        thermal_obstacle = cv2.inRange(
            blur,
            road_max + 1,
            255
        )

        # ======================================
        # FLOOR PRIORITY MASK
        # ======================================

        mask = np.zeros_like(semantic)

        cv2.rectangle(
            mask,
            (0, int(semantic.shape[0]*0.25)),
            (semantic.shape[1], semantic.shape[0]),
            255,
            -1
        )

        semantic = cv2.bitwise_and(
            semantic,
            mask
        )

        thermal_obstacle = cv2.bitwise_and(
            thermal_obstacle,
            mask
        )

        # morphological closing - menghubungkan area jalan
        semantic = cv2.morphologyEx(
            semantic,
            cv2.MORPH_CLOSE,
            np.ones((5,5), np.uint8)
        )

        thermal_obstacle = cv2.morphologyEx(
            thermal_obstacle,
            cv2.MORPH_OPEN,
            np.ones((3,3), np.uint8)
        )

        # # morphological opening - hapus noise kecil (human/cat)
        # semantic = cv2.morphologyEx(
        #     semantic,
        #     cv2.MORPH_OPEN,
        #     np.ones((5,5), np.uint8)
        # )

        # ======================================
        # COLORIZE TRAVERSABLE
        # ======================================

        # ======================================
        # COLORIZE 3 KELAS
        # ======================================
        # LANTAI (traversable, abu sedang) -> HIJAU
        # OBJEK PANAS (manusia/hewan, putih)-> MERAH  <-- yang dihindari thermal
        # TEMBOK / dingin (gelap)          -> HITAM
        # ======================================

        color_semantic = np.zeros(
            (semantic.shape[0], semantic.shape[1], 3),
            dtype=np.uint8
        )

        # lantai bisa dilewati = hijau
        color_semantic[semantic > 100] = (109, 237, 115)

        # objek panas = MERAH (digambar terakhir supaya menang dari lantai)
        color_semantic[thermal_obstacle > 100] = (0, 0, 255)

        # sisanya (tembok / dingin) tetap hitam (sudah 0 dari np.zeros)

        cv2.imshow(
            "RAW THERMAL",
            frame
        )

        cv2.imshow(
            "SEMANTIC COLOR",
            color_semantic
        )

        cv2.waitKey(1)

        ros_img = self.bridge.cv2_to_imgmsg(
            color_semantic,
            encoding='bgr8'
        )

        obstacle_img = self.bridge.cv2_to_imgmsg(
            thermal_obstacle,
            encoding='mono8'
        )

        self.pub.publish(ros_img)
        self.obstacle_pub.publish(obstacle_img)

if __name__ == "__main__":

    rospy.init_node("semantic_thermal")

    SemanticThermal()

    rospy.spin()
