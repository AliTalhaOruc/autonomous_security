#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped

from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

import tf2_ros
from tf2_geometry_msgs import do_transform_point
import math

class TrackedTarget:
    def __init__(self, target_id, class_name, threat_score):
        self.target_id = target_id
        self.class_name = class_name
        self.threat_score = threat_score
        self.observations = []
        self.is_locked = False
        self.final_x = 0.0
        self.final_y = 0.0

class YoloThreatDetector(Node):
    def __init__(self):
        super().__init__('yolo_threat_detector')
        self.frame_count = 0
        # Genel ve Sınıfa Özel Güven Eşikleri (False Positive Engelleme)
        self.default_conf_threshold = 0.60
        self.class_conf_thresholds = {
            'house': 0.80,       # Duvarları ev sanmasını engellemek için yukseltildi
            'fire_station': 0.80,
            'soldier': 0.68,
            'tank': 0.75,
            'stop_sign': 0.60
        }

        self.threat_map = {
            'soldier': 0.70,
            'fire_station': 0.50,
            'house': 0.60,
            'stop_sign': 0.00,
            'tank': 0.95
        }

        # Sınıfa Özel Esleme Mesafeleri (Coklu Marker Basımını Engelleme)
        self.class_match_thresholds = {
            'tank': 5.9,
            'house': 10.0,
            'fire_station': 10.0,
            'soldier': 1.5,
            'stop_sign': 1.2
        }

        self.CONFIRMATION_THRESHOLD = 5
        self.MIN_DEPTH_M = 0.5
        self.MAX_DEPTH_M = 12.0

        self.pending_targets = []
        self.locked_targets = []
        self.global_id_counter = 1

        self.get_logger().info('=== [ADIM 1] YOLO Modeli yukleniyor... ===')
        
        
        pkg_share = get_package_share_directory('autonomous_security')
        model_path = os.path.join(pkg_share, 'models', 'best.pt')
        
        # Modeli yükleme ve GPU'ya taşıma
        self.model = YOLO(model_path)
        self.model.to('cuda')
        
        self.bridge = CvBridge()
        self.get_logger().info(f'=== [ADIM 1] Model Başarıyla Yüklendi: {model_path} ===')

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.camera_frame_id = "camera_color_optical_frame"
        self.latest_depth_image = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.sub_info = self.create_subscription(
            CameraInfo, '/camera/camera/camera_info', self.camera_info_callback, 10)
        self.sub_depth = self.create_subscription(
            Image, '/camera/camera/depth/image_raw', self.depth_callback, 10)
        self.sub_rgb = self.create_subscription(
            Image, '/camera/camera/image_raw', self.rgb_callback, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/detected_threats', 10)
        self.get_logger().info('=== [ADIM 2] Düğüm başlatıldı. Kamera bekleniyor... ===')

    def camera_info_callback(self, msg: CameraInfo):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.camera_frame_id = msg.header.frame_id

    def depth_callback(self, msg: Image):
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Derinlik donusum hatasi: {e}')

    def rgb_callback(self, msg: Image):
        self.frame_count += 1
        if self.frame_count % 2 != 0:  # Her 2 kareden 1'ini işle (FPS'i yarıya bölüp GPU'yu rahatlatır)
            return
        if self.fx is None or self.latest_depth_image is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            return

        results = self.model(cv_image, verbose=False)[0]

        for box in results.boxes:
            conf = float(box.conf[0])
            cls_id = int(box.cls[0])
            class_name = self.model.names[cls_id]

            # Sınıfa özel güven eşiği kontrolü
            min_conf = self.class_conf_thresholds.get(class_name, self.default_conf_threshold)
            if conf < min_conf:
                continue

            x1, y1, x2, y2 = map(int, box.xyxy[0])
            u_center = int((x1 + x2) / 2)
            v_center = int((y1 + y2) / 2)


            # [YENİ EKLEME] STANDART SAPMA (DEPTH VARIANCE) FILTRESI
            # Tüm Bounding Box içerisindeki derinlik yamasını kesiyoruz
            full_depth_patch = self.latest_depth_image[
                max(0, y1):min(self.latest_depth_image.shape[0], y2),
                max(0, x1):min(self.latest_depth_image.shape[1], x2)
            ]
            
            # Geçerli derinlik piksellerini filtrele
            valid_full_depths = full_depth_patch[np.isfinite(full_depth_patch) & (full_depth_patch > 0)].astype(np.float32)
            
            if len(valid_full_depths) > 0:
                # Kamera verisi mm cinsinden ise metreye çeviriyoruz
                if np.median(valid_full_depths) > 100:
                    valid_full_depths /= 1000.0
                
                # Bounding Box içi derinlik standart sapması
                depth_std = float(np.std(valid_full_depths))
                
                # Sadece 'house' sınıfı için varyans filtresi uygula
                if class_name == "house":
                    # Belirlediğimiz güvenli sınır: 0.25
                    if depth_std < 0.25:
                        self.get_logger().warn(
                            f"[ELENDI - DUVAR] Hatalı Ev Tespiti! Std: {depth_std:.3f} < 0.25 | BBox: [{x1},{y1},{x2},{y2}]"
                        )
                        continue  # Bu tespiti atla, haritaya marker basma!
                    else:
                        self.get_logger().info(
                            f"[ONAYLANDI - EV] Gerçek Ev! Std: {depth_std:.3f} >= 0.25"
                        )

            # Mevcut kodunuz: Hedef tespiti ve orta nokta hesabı için küçük ROI kesimi
            h_roi = max(1, int((y2 - y1) * 0.1))
            w_roi = max(1, int((x2 - x1) * 0.1))
            depth_patch = self.latest_depth_image[
                max(0, v_center - h_roi):min(self.latest_depth_image.shape[0], v_center + h_roi),
                max(0, u_center - w_roi):min(self.latest_depth_image.shape[1], u_center + w_roi)
            ]

            valid_depths = depth_patch[np.isfinite(depth_patch) & (depth_patch > 0)]
            if len(valid_depths) == 0:
                continue

            depth_m = np.median(valid_depths)
            if depth_m > 100:  
                depth_m /= 1000.0

            if depth_m < self.MIN_DEPTH_M or depth_m > self.MAX_DEPTH_M:
                continue

            X_c = (u_center - self.cx) * depth_m / self.fx
            Y_c = (v_center - self.cy) * depth_m / self.fy
            Z_c = depth_m

            point_in_map = self.transform_to_map(X_c, Y_c, Z_c, msg.header.stamp)

            if point_in_map is not None:
                map_x = point_in_map.point.x
                map_y = point_in_map.point.y
                self.process_detection(class_name, conf, map_x, map_y, depth_m)

        self.publish_locked_markers()


    def process_detection(self, class_name, conf, x, y, depth_m):
        match_radius = self.class_match_thresholds.get(class_name, 2.0)

        # A) Kilitli hedeflerle eşleştirme (Çoklu marker engelleme)
        for locked in self.locked_targets:
            dist = math.hypot(x - locked.final_x, y - locked.final_y)
            if dist < match_radius:
                # Zaten kilitlenmiş bir hedefin kapsama alanında, yeni marker EKLEME.
                return

        # B) Bekleyen adaylarla eşleştirme
        matched_candidate = None
        for candidate in self.pending_targets:
            if candidate.class_name == class_name:
                avg_x = sum(p[0] for p in candidate.observations) / len(candidate.observations)
                avg_y = sum(p[1] for p in candidate.observations) / len(candidate.observations)
                dist = math.hypot(x - avg_x, y - avg_y)
                
                if dist < match_radius:
                    matched_candidate = candidate
                    break

        if matched_candidate is not None:
            matched_candidate.observations.append((x, y))
            
            if len(matched_candidate.observations) >= self.CONFIRMATION_THRESHOLD:
                matched_candidate.final_x = sum(p[0] for p in matched_candidate.observations) / len(matched_candidate.observations)
                matched_candidate.final_y = sum(p[1] for p in matched_candidate.observations) / len(matched_candidate.observations)
                matched_candidate.is_locked = True

                self.locked_targets.append(matched_candidate)
                self.pending_targets.remove(matched_candidate)

                print(f"[KİLİTLENDİ] {matched_candidate.class_name.upper()}_{matched_candidate.target_id:03d} -> X:{matched_candidate.final_x:.2f}, Y:{matched_candidate.final_y:.2f}")

        else:
            threat_score = self.threat_map.get(class_name, 0.40)
            new_target = TrackedTarget(self.global_id_counter, class_name, threat_score)
            new_target.observations.append((x, y))
            self.pending_targets.append(new_target)
            self.global_id_counter += 1

    def publish_locked_markers(self):
        marker_array = MarkerArray()

        for target in self.locked_targets:
            # 1. Silindir Marker
            cyl_marker = Marker()
            cyl_marker.header.frame_id = "map"
            cyl_marker.header.stamp = self.get_clock().now().to_msg()
            cyl_marker.ns = "threat_cylinders"
            cyl_marker.id = target.target_id
            cyl_marker.type = Marker.CYLINDER
            cyl_marker.action = Marker.ADD
            cyl_marker.pose.position.x = target.final_x
            cyl_marker.pose.position.y = target.final_y
            cyl_marker.pose.position.z = 0.2
            cyl_marker.pose.orientation.w = 1.0
            cyl_marker.scale.x = 0.6
            cyl_marker.scale.y = 0.6
            cyl_marker.scale.z = 0.4
            cyl_marker.color.r = float(target.threat_score)
            cyl_marker.color.g = float(1.0 - target.threat_score)
            cyl_marker.color.b = 0.0
            cyl_marker.color.a = 0.8
            marker_array.markers.append(cyl_marker)

            # 2. Metin (Text) Marker
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "threat_labels"
            text_marker.id = target.target_id + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = target.final_x
            text_marker.pose.position.y = target.final_y
            text_marker.pose.position.z = 1.0
            text_marker.scale.z = 0.4
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = f"[{target.class_name.upper()}_{target.target_id:03d}]\nThreat: %{int(target.threat_score * 100)}"
            marker_array.markers.append(text_marker)

        if len(marker_array.markers) > 0:
            self.marker_pub.publish(marker_array)

    def transform_to_map(self, x, y, z, stamp):
        pt = PointStamped()
        pt.header.stamp = stamp
        pt.header.frame_id = self.camera_frame_id
        pt.point.x = float(x)
        pt.point.y = float(y)
        pt.point.z = float(z)

        try:
            transform = self.tf_buffer.lookup_transform('map', self.camera_frame_id, rclpy.time.Time(), timeout=Duration(seconds=0.2))
            return do_transform_point(pt, transform)
        except Exception:
            return None

def main(args=None):
    rclpy.init(args=args)
    node = YoloThreatDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()