#!/usr/bin/env python3
# ============================================================================
# thermal_blindspot_to_scan.py   (PENGGANTI thermal_blindspot_to_cloud.py)
# ----------------------------------------------------------------------------
# Kenapa diganti dari PointCloud ke LaserScan?
#   * PointCloud (versi lama) cuma MENANDAI titik, tidak otomatis menghapus.
#     Akibatnya blob thermal yang salah (tembok jauh terbaca "panas") nyangkut
#     jadi OBSTACLE HANTU di costmap, padahal di Gazebo kosong.
#   * LaserScan otomatis CLEAR: beam tanpa deteksi = inf -> costmap
#     membersihkan jalur beam itu. Jadi begitu objek tak terlihat lagi,
#     tandanya langsung hilang. Tidak ada hantu yang nyangkut.
#
# Anti false-positive (biar tembok TIDAK ditandai sebagai kucing):
#   1. Hanya pakai BAGIAN BAWAH gambar (objek dekat di tanah). Tembok jauh ada
#      di bagian atas -> diabaikan.
#   2. Range dibatasi pendek (default <= 0.85 m). Blind-spot itu soal objek
#      DEKAT yang tak terlihat LiDAR; objek jauh bukan urusan node ini.
#   3. Blob harus cukup besar (bukan noise) TAPI tidak boleh raksasa
#      (blob raksasa = dinding, bukan kucing).
#
# Topik keluaran: /thermal_blindspot_scan  (sensor_msgs/LaserScan, frame base_link)
# Dipakai costmap lewat common_costmap.yaml (source 'thermal_blindspot').
#
# KALIBRASI (lihat di RViz: Add -> LaserScan -> /thermal_blindspot_scan):
#   saat kucing TEPAT di depan, beam merah harus muncul di posisi kucing.
#   Saat depan KOSONG, harus TIDAK ada beam (semua inf). Kalau pas kosong
#   masih ada beam -> kecilkan far_range / naikkan roi_top_frac / naikkan
#   min_blob_px.
# ============================================================================

import math
import rospy
import numpy as np
import cv2

from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge


class ThermalBlindspotToScan:

    def __init__(self):
        self.bridge = CvBridge()

        self.frame_id     = rospy.get_param("~frame_id", "base_link")
        # --- proyeksi (TUNE sambil lihat RViz) ---
        self.near_range   = rospy.get_param("~near_range", 0.25)   # m (baris bawah)
        self.far_range    = rospy.get_param("~far_range", 0.85)    # m (batas atas ROI) -- SENGAJA pendek
        self.lateral_fov  = rospy.get_param("~lateral_fov", 0.70)  # rad, setengah lebar
        self.roi_top_frac = rospy.get_param("~roi_top_frac", 0.45) # buang 45% atas (tembok jauh)
        # --- gating anti false-positive ---
        self.pixel_thresh = rospy.get_param("~pixel_thresh", 100)
        self.min_blob_px  = rospy.get_param("~min_blob_px", 40)    # buang noise kecil
        # Hanya proses blob yang TITIK-TENGAHnya benar2 di area dekat (bawah).
        # Ini mencegah bercak JAUH (tembok/struktur terang di atas gambar) yang
        # ujungnya sedikit menjorok ke ROI ikut ditandai jadi obstacle hantu.
        self.min_centroid_frac = rospy.get_param("~min_centroid_frac", 0.60)
        # CATATAN: tidak ada penolakan "blob raksasa = tembok" lagi.
        # Tembok itu DINGIN (hitam), tidak pernah masuk mask panas. Objek panas
        # yang DEKAT memang tampil sebagai blob besar -> JUSTRU harus dideteksi.
        # --- bentuk scan ---
        self.num_beams    = rospy.get_param("~num_beams", 61)
        self.range_max    = rospy.get_param("~range_max", 1.2)
        self.range_min    = rospy.get_param("~range_min", 0.05)

        # Memory obstacle thermal
        self.memory_duration = rospy.get_param("~memory_duration", 0.7)

        self.last_ranges = None
        self.last_detect_time = rospy.Time(0)

        self.pub = rospy.Publisher("/thermal_blindspot_scan", LaserScan, queue_size=1)
        rospy.Subscriber("/thermal_obstacle_mask", Image, self.cb, queue_size=1)

        rospy.loginfo(
            "thermal_blindspot_to_scan siap | near=%.2f far=%.2f fov=%.2f roi_top=%.2f",
            self.near_range, self.far_range, self.lateral_fov, self.roi_top_frac
        )

    # ------------------------------------------------------------------
    def cb(self, msg):
        mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        h, w = mask.shape

        _, mask = cv2.threshold(mask, self.pixel_thresh, 255, cv2.THRESH_BINARY)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  np.ones((3, 3), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

        top = int(h * self.roi_top_frac)        # hanya pakai bawah ROI
        total_px = float(h * w)

        angle_min = -self.lateral_fov
        angle_max =  self.lateral_fov
        angle_inc = (angle_max - angle_min) / (self.num_beams - 1)

        ranges = [float('inf')] * self.num_beams  # default: KOSONG (akan di-clear costmap)

        n_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)

        centroid_line = int(h * self.min_centroid_frac)

        for lbl in range(1, n_labels):
            area = stats[lbl, cv2.CC_STAT_AREA]
            if area < self.min_blob_px:
                continue

            # tolak blob yang titik-tengahnya masih di area jauh (atas)
            cy = centroids[lbl][1]
            if cy < centroid_line:
                continue

            ys, xs = np.where(labels == lbl)
            keep = ys >= top                            # hanya bagian bawah (objek dekat)
            ys, xs = ys[keep], xs[keep]
            if len(xs) == 0:
                continue

            for i in range(0, len(xs), 4):
                px, py = xs[i], ys[i]

                v = (py - top) / float(max(1, h - top))            # 0 atas-ROI .. 1 bawah
                rng = self.far_range + (self.near_range - self.far_range) * v
                if rng > self.far_range:
                    rng = self.far_range

                u = (px - w / 2.0) / (w / 2.0)                     # -1..+1
                ang = -u * self.lateral_fov                        # kanan gambar -> kanan robot (-)

                beam = int(round((ang - angle_min) / angle_inc))
                if 0 <= beam < self.num_beams:
                    if rng < ranges[beam]:
                        ranges[beam] = rng

        n_hit = sum(1 for r in ranges if math.isfinite(r))

        now = rospy.Time.now()

        if n_hit > 0:
            self.last_ranges = ranges[:]
            self.last_detect_time = now

        elif self.last_ranges is not None:
            dt = (now - self.last_detect_time).to_sec()

            if dt < self.memory_duration:
                ranges = self.last_ranges[:]
                n_hit = sum(1 for r in ranges if math.isfinite(r))

        self.publish_scan(ranges, angle_min, angle_inc, msg.header.stamp)

        rospy.loginfo_throttle(
            1.0, "THERMAL BLINDSPOT SCAN: %d/%d beam", n_hit, self.num_beams
        )

        if n_hit == 0 and self.last_ranges is not None:
            rospy.loginfo_throttle(
                1.0, "THERMAL MEMORY ACTIVE"
            )

    # ------------------------------------------------------------------
    def publish_scan(self, ranges, angle_min, angle_inc, stamp):
        scan = LaserScan()
        scan.header.stamp = stamp if stamp != rospy.Time(0) else rospy.Time.now()
        scan.header.frame_id = self.frame_id
        scan.angle_min = angle_min
        scan.angle_max = angle_min + angle_inc * (len(ranges) - 1)
        scan.angle_increment = angle_inc
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges
        scan.intensities = []
        self.pub.publish(scan)


def main():
    rospy.init_node("thermal_blindspot_to_scan")
    ThermalBlindspotToScan()
    rospy.spin()


if __name__ == "__main__":
    main()
