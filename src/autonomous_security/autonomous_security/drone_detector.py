#!/usr/bin/env python3
import os
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory

class DroneDetector(Node):
    def __init__(self):
        super().__init__('drone_detector')

        # ROS 2 Parametre Tanımlamaları (İstenirse launch dosyasından kolayca değiştirilebilir)
        self.declare_parameter('confidence_threshold', 0.25)
        self.conf_thresh = self.get_parameter('confidence_threshold').get_parameter_value().double_value

        # Yeni Mimariye Uygun Topic Yapılanması
        # Keşif / RGBD sisteminden tamamen bağımsız Anti-Drone Taret Kamerası
        self.subscription = self.create_subscription(
            Image,
            '/anti_drone/anti_drone_camera/image_raw',
            self.image_callback,
            10
        )
        
        # Takip & FSM için hedef piksel ve işlenmiş görüntü yayıncıları
        self.target_pub = self.create_publisher(Point, '/drone_target_pixel', 10)
        self.debug_image_pub = self.create_publisher(Image, '/anti_drone/processed_image', 10)

        self.bridge = CvBridge()

        # Model Ağırlığı Yükleme (Package Share üzerinden)
        try:
            pkg_share = get_package_share_directory('autonomous_security')
            model_path = os.path.join(pkg_share, 'models', 'drone_best.pt')
            self.model = YOLO(model_path)
            self.get_logger().info(f"YOLO Anti-Drone Modeli Yüklendi: {model_path}")
        except Exception as e:
            self.get_logger().error(f"YOLO Model Yükleme Hatası: {str(e)}")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CvBridge Dönüştürme Hatası: {str(e)}")
            return

        # Görüntü Boyutları ve Ekran Merkezi Hesabı
        height, width, _ = cv_image.shape
        center_x, center_y = width // 2, height // 2

        # Kamera merkezine Crosshair çizimi
        cv2.drawMarker(cv_image, (center_x, center_y), (255, 255, 255), cv2.MARKER_CROSS, 20, 1)

        # YOLO Tahmini (conf filtresi ile)
        results = self.model(cv_image, conf=self.conf_thresh, verbose=False)

        # Eğer sahnede tespit edilen nesneler/dronelar varsa
        if len(results) > 0 and len(results[0].boxes) > 0:
            boxes = results[0].boxes

            # En yüksek confidence (güven) değerine sahip kutuyu seç
            best_box = max(boxes, key=lambda b: float(b.conf[0]))

            # Koordinatları al
            xyxy = best_box.xyxy[0].cpu().numpy()
            xmin, ymin, xmax, ymax = map(int, xyxy)
            conf = float(best_box.conf[0])

            # Merkez piksel koordinatları
            cx = int((xmin + xmax) / 2)
            cy = int((ymin + ymax) / 2)

            # Görselleştirme (Bounding Box + Merkez Noktası + Bilgi Metni)
            cv2.rectangle(cv_image, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
            cv2.circle(cv_image, (cx, cy), 5, (0, 0, 255), -1)
            cv2.putText(
                cv_image, 
                f"DRONE: {conf:.2f}", 
                (xmin, ymin - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 
                0.5, 
                (0, 255, 0), 
                2
            )

            # Hedef Piksel Mesajını Hazırla ve Yayınla
            # x = cx (pixel_x), y = cy (pixel_y), z = conf (confidence)
            target_msg = Point()
            target_msg.x = float(cx)
            target_msg.y = float(cy)
            target_msg.z = float(conf)
            self.target_pub.publish(target_msg)

        # Debug/İşlenmiş Görüntüyü Yayınla
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