#!/usr/bin/env python3
import os
import cv2
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray  
from cv_bridge import CvBridge
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
from collections import deque

class DroneDetector(Node):
    def __init__(self):
        super().__init__('drone_detector')

        self.declare_parameter('confidence_threshold', 0.65)
        self.conf_thresh = self.get_parameter('confidence_threshold').get_parameter_value().double_value

        self.subscription = self.create_subscription(
            Image,
            '/anti_drone/anti_drone_camera/image_raw',
            self.image_callback,
            10
        )
        
        self.sub_depth = self.create_subscription(
            Image, 
            '/camera/camera/depth/image_raw', 
            self.depth_callback, 
            10
        )
        
        self.target_pub = self.create_publisher(Float64MultiArray, '/drone_target_data', 10)
        self.debug_image_pub = self.create_publisher(Image, '/anti_drone/processed_image', 10)

        self.bridge = CvBridge()
        self.latest_depth_image = None  
        self.depth_window = deque(maxlen=7) # Son 7 kare hafızası

        try:
            pkg_share = get_package_share_directory('autonomous_security')
            model_path = os.path.join(pkg_share, 'models', 'drone_best.pt')
            self.model = YOLO(model_path)
            self.get_logger().info(f"YOLO Modeli Yüklendi: {model_path}")
        except Exception as e:
            self.get_logger().error(f"YOLO Model Yükleme Hatası: {str(e)}")

    def depth_callback(self, msg: Image):
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Derinlik donusum hatasi: {e}')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CvBridge Dönüştürme Hatası: {str(e)}")
            return

        height, width, _ = cv_image.shape
        center_x, center_y = width // 2, height // 2
        cv2.drawMarker(cv_image, (center_x, center_y), (255, 255, 255), cv2.MARKER_CROSS, 20, 1)

        results = self.model(cv_image, conf=self.conf_thresh, verbose=False)

        if len(results) > 0 and len(results[0].boxes) > 0:
            boxes = results[0].boxes
            best_box = max(boxes, key=lambda b: float(b.conf[0]))

            xyxy = best_box.xyxy[0].cpu().numpy()
            xmin, ymin, xmax, ymax = map(int, xyxy)
            conf = float(best_box.conf[0])

            cx = int((xmin + xmax) / 2)
            cy = int((ymin + ymax) / 2)

            current_frame_depth = -1.0
            if self.latest_depth_image is not None:
                try:
                    # Kutu içerisindeki derinlik piksellerini analiz et
                    roi = self.latest_depth_image[ymin:ymax, xmin:xmax]
                    # Sonsuz, NaN ve sıfır olmayan pikselleri ayıkla
                    valid_roi = roi[(roi > 0.1) & (roi < 100.0) & (~np.isinf(roi)) & (~np.isnan(roi))]
                    
                    if valid_roi.size > 0:
                        # Hatalı uç değerleri ezmek için medyanı (orta değeri) alıyoruz
                        current_frame_depth = float(np.median(valid_roi))
                except Exception:
                    pass

            # Karede derinlik yakalandıysa kayan pencereye ekle
            if current_frame_depth > 0.1:
                self.depth_window.append(current_frame_depth)

            # Son 7 karenin medyanını alarak gürültüyü tamamen sıfırlıyoruz
            if len(self.depth_window) > 0:
                filtered_depth = float(np.median(self.depth_window))
            else:
                filtered_depth = -1.0

            # Görselleştirme
            cv2.rectangle(cv_image, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
            cv2.circle(cv_image, (cx, cy), 5, (0, 0, 255), -1)
            cv2.putText(
                cv_image, 
                f"DRONE: {conf:.2f} | Dist: {filtered_depth:.2f}m", 
                (xmin, ymin - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2
            )

            target_msg = Float64MultiArray()
            target_msg.data = [float(cx), float(cy), float(filtered_depth), float(conf)]
            self.target_pub.publish(target_msg)

        processed_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        self.debug_image_pub.publish(processed_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DroneDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

