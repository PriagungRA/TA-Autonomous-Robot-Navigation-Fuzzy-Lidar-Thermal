#!/usr/bin/env python3
# ============================================================================
# thermal_blindspot_to_cloud.py
# ----------------------------------------------------------------------------
# TUJUAN (inti skripsi: fusi LiDAR + thermal):
#   LiDAR 1D punya blind spot vertikal -> objek pendek (kucing) yang ada DI
#   BAWAH bidang scan tidak terdeteksi, jadi ditabrak/ditembus.
#   Kamera thermal melihat ke bawah-depan dan BISA melihat objek itu.
#
#   Node ini mengambil /thermal_obstacle_mask (objek panas hasil
#   semantic_thermal.py), memproyeksikan blob-nya ke titik di TANAH di depan
#   robot, lalu menerbitkannya sebagai sensor_msgs/PointCloud2.
#   Costmap (lihat common_costmap.yaml -> source 'thermal_blindspot') akan
#   menandai titik itu sebagai obstacle, sehingga APF menghindarinya dengan
#   logika yang SAMA seperti menghindari tembok dari LiDAR -> mulus.
#
#   Ini cara fusi yang "bersih" & mudah dipertahankan di sidang:
#   "thermal mengisi blind spot LiDAR dengan menyumbang titik obstacle ke
#    costmap melalui proyeksi ground-plane terkalibrasi."
#
# CARA PAKAI:
#   Sudah otomatis di-launch lewat nav_sim.launch (grup use_thermal_nav).
#   Kalau jalan manual:  rosrun fuzzy_thermal_nav thermal_blindspot_to_cloud.py
#
# KALIBRASI (paling penting - sesuaikan sambil lihat kucing di RViz):
#   ~near_range, ~far_range  -> jarak (m) untuk baris bawah & atas ROI
#   ~lateral_fov             -> setengah lebar sebaran kiri-kanan (rad)
#   ~point_height            -> ketinggian titik (≈ tinggi bidang LiDAR)
#   ~roi_top_frac            -> abaikan bagian atas gambar (langit/jauh)
# ============================================================================

import rospy
import numpy as np
import cv2
import struct

from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header
from cv_bridge import CvBridge


class ThermalBlindspotToCloud:

    def __init__(self):
        self.bridge = CvBridge()

        # ---- Frame tujuan publish (harus ada di TF). base_link aman. ----
        self.frame_id = rospy.get_param("~frame_id", "base_link")

        # ---- Kalibrasi proyeksi (TUNE INI sambil lihat RViz) ----
        self.near_range   = rospy.get_param("~near_range", 0.30)   # m, baris bawah ROI
        self.far_range    = rospy.get_param("~far_range", 1.40)    # m, baris atas ROI
        self.lateral_fov  = rospy.get_param("~lateral_fov", 0.70)  # rad, setengah sebaran kiri/kanan
        self.point_height = rospy.get_param("~point_height", 0.30) # m, ≈ tinggi bidang LiDAR
        self.roi_top_frac = rospy.get_param("~roi_top_frac", 0.25) # abaikan 25% atas gambar
        self.pixel_thresh = rospy.get_param("~pixel_thresh", 100)  # ambang mask (mono8)
        self.min_blob_px  = rospy.get_param("~min_blob_px", 30)    # buang noise kecil
        self.step         = rospy.get_param("~sample_step", 6)     # subsampling piksel

        self.pub = rospy.Publisher(
            "/thermal_blindspot_cloud",
            PointCloud2,
            queue_size=1
        )

        rospy.Subscriber(
            "/thermal_obstacle_mask",
            Image,
            self.callback,
            queue_size=1
        )

        rospy.loginfo(
            "thermal_blindspot_to_cloud siap | near=%.2f far=%.2f fov=%.2f h=%.2f frame=%s",
            self.near_range, self.far_range, self.lateral_fov,
            self.point_height, self.frame_id
        )

    # ------------------------------------------------------------------
    def callback(self, msg):
        mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")

        h, w = mask.shape

        # Buang noise & rapatkan blob
        _, mask = cv2.threshold(mask, self.pixel_thresh, 255, cv2.THRESH_BINARY)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

        # Hanya pakai bagian bawah gambar (tanah depan), bukan langit/jauh
        top = int(h * self.roi_top_frac)

        points = []

        # Pakai connected components untuk buang blob kecil (noise)
        n_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)

        for lbl in range(1, n_labels):
            if stats[lbl, cv2.CC_STAT_AREA] < self.min_blob_px:
                continue

            ys, xs = np.where(labels == lbl)
            keep = ys >= top
            ys, xs = ys[keep], xs[keep]
            if len(xs) == 0:
                continue

            # subsampling biar cloud tidak terlalu padat
            for i in range(0, len(xs), self.step):
                px = xs[i]
                py = ys[i]

                # baris (v): bawah gambar = dekat, atas = jauh
                v = (py - top) / float(max(1, h - top))   # 0 (atas ROI) .. 1 (bawah)
                rng = self.far_range + (self.near_range - self.far_range) * v

                # kolom (u): kiri/kanan -> sudut lateral
                u = (px - w / 2.0) / (w / 2.0)             # -1 .. +1
                ang = u * self.lateral_fov

                # base_link: x maju, y kiri(+). kolom kanan gambar -> y negatif.
                fx = rng * np.cos(ang)
                fy = -rng * np.sin(ang)
                fz = self.point_height

                points.append((fx, fy, fz))

        self.publish_cloud(points, msg.header.stamp)

        rospy.loginfo_throttle(
            1.0,
            "THERMAL BLINDSPOT: %d titik obstacle dikirim ke costmap",
            len(points)
        )

    # ------------------------------------------------------------------
    def publish_cloud(self, points, stamp):
        header = Header()
        header.stamp = stamp if stamp != rospy.Time(0) else rospy.Time.now()
        header.frame_id = self.frame_id

        fields = [
            PointField('x', 0,  PointField.FLOAT32, 1),
            PointField('y', 4,  PointField.FLOAT32, 1),
            PointField('z', 8,  PointField.FLOAT32, 1),
        ]

        buf = bytearray()
        for (x, y, z) in points:
            buf += struct.pack('fff', x, y, z)

        cloud = PointCloud2()
        cloud.header = header
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = fields
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = 12 * len(points)
        cloud.is_dense = True
        cloud.data = bytes(buf)

        self.pub.publish(cloud)


def main():
    rospy.init_node("thermal_blindspot_to_cloud")
    ThermalBlindspotToCloud()
    rospy.spin()


if __name__ == "__main__":
    main()
