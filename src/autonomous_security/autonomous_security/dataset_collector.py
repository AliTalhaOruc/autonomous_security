#!/usr/bin/env python3
import os
import time
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class DatasetCollector(Node):
    def __init__(self):
        super().__init__('dataset_collector')

        # Parametreler: Kayıt klasörü ve kayıt aralığı (saniye)
        self.declare_parameter('save_dir', 'drone_dataset_raw')
        self.declare_parameter('interval', 0.5)  # Yarım saniyede bir foto

        self.save_dir = self.get_parameter('save_dir').get_parameter_value().string_value
        self.interval = self.get_parameter('interval').get_parameter_value().double_value

        # Kayıt klasörünü oluştur
        self.full_save_path = os.path.expanduser(os.path.join('~', self.save_dir))
        os.makedirs(self.full_save_path, exist_ok=True)

        # Kamera aboneliği (Taret kamerasını dinler)
        self.subscription = self.create_subscription(
            Image,
            '/anti_drone/anti_drone_camera/image_raw',
            self.image_callback,
            10
        )

        self.bridge = CvBridge()
        self.last_save_time = time.time()
        self.img_counter = 0

        self.get_logger().info(f"Dataset Toplayıcı Başlatıldı!")
        self.get_logger().info(f"Görseller buraya kaydedilecek: {self.full_save_path}")
        self.get_logger().info(f"Kayıt aralığı: {self.interval} saniye")

    def image_callback(self, msg):
        current_time = time.time()

        # Belirlenen süre (0.5 sn) dolduysa fotoğrafı kaydet
        if current_time - self.last_save_time >= self.interval:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except Exception as e:
                self.get_logger().error(f"Görüntü dönüştürme hatası: {str(e)}")
                return

            self.img_counter += 1
            file_name = f"drone_near_{int(time.time())}_{self.img_counter:04d}.jpg"
            file_path = os.path.join(self.full_save_path, file_name)

            cv2.imwrite(file_path, cv_image)
            self.last_save_time = current_time

            self.get_logger().info(f"[{self.img_counter}] Fotoğraf kaydedildi: {file_name}")

def main(args=None):
    rclpy.init(args=args)
    node = DatasetCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()