import math
import numpy as np
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
import tf2_ros


class AgresifSahaExplorer(Node):

    def __init__(self):
        super().__init__('agresif_saha_explorer')

        # --- DURUM (STATE) ---
        self.state = "STARTUP_MOVE" 
        self.start_time = self.get_clock().now()

        self.map_data = None
        self.map_info = None
        self.blacklist = []
        self.current_goal = None
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # --- PARAMETRELER ---
        self.min_cluster_size = 8     # Çok küçük gürültü noktalarını eler

        # --- ROS 2 BAĞLANTILARI ---
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # CPU'yu boğmamak için döngüyü 1.0 saniyede 1 kez çalıştırıyoruz
        self.timer = self.create_timer(1.0, self.main_loop)

        self.get_logger().info("⚔️ Optimize Edilmiş Keşif Modülü Başlatıldı!")

    def map_callback(self, msg):
        self.map_data = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width)
        )
        self.map_info = msg.info

    def main_loop(self):
        if self.map_data is None or self.map_info is None:
            return

        if self.state == "STARTUP_MOVE":
            self.startup_logic()
        elif self.state == "IDLE":
            self.explore()

    def startup_logic(self):
        self.stop_robot()
        self.state = "IDLE"
        self.get_logger().info("🚀 Başlangıç atılımı tamamlandı, tarama başlıyor...")

    def explore(self):
        try:
            # TF zaman aşımına düşmemesi için 1.5 saniye tolerans
            t = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(), timeout=Duration(seconds=1.5))
            self.robot_x = t.transform.translation.x
            self.robot_y = t.transform.translation.y
            qx = t.transform.rotation.x
            qy = t.transform.rotation.y
            qz = t.transform.rotation.z
            qw = t.transform.rotation.w

            self.robot_yaw = math.atan2(
                2.0 * (qw * qz + qx * qy),
                1.0 - 2.0 * (qy * qy + qz * qz)
            )
        except Exception as e:
            self.get_logger().warn(f"TF dönüşüm bekleniyor... ({e})")
            return

        clusters = self.find_frontier_clusters_fast()

        if not clusters:
            self.get_logger().info("🏁 Keşfedilecek alan kalmadı! Görev tamamlandı.")
            self.stop_robot()
            self.state = "FINISHED"
            return

        target = self.select_best_cluster(clusters)
        if target:
            self.send_goal(target[0], target[1])
        else:
            self.get_logger().warn("Gidilebilir uygun frontier bulunamadı, bekleniyor...")

    # ---------------------------------------------------
    # HIZLANDIRILMIŞ VE VEKTÖRLEŞTİRİLMİŞ FRONTIER BULUCU (NUMPY)
    # ---------------------------------------------------
    def find_frontier_clusters_fast(self):
        free_mask = (self.map_data == 0)
        unknown_mask = (self.map_data == -1)

        has_unknown_neighbor = np.zeros_like(free_mask, dtype=bool)
        has_unknown_neighbor[1:, :] |= unknown_mask[:-1, :]  # Üst
        has_unknown_neighbor[:-1, :] |= unknown_mask[1:, :]  # Alt
        has_unknown_neighbor[:, 1:] |= unknown_mask[:, :-1]  # Sol
        has_unknown_neighbor[:, :-1] |= unknown_mask[:, 1:]  # Sağ

        frontier_mask = free_mask & has_unknown_neighbor

        frontier_mask[0, :] = frontier_mask[-1, :] = False
        frontier_mask[:, 0] = frontier_mask[:, -1] = False

        y_indices, x_indices = np.where(frontier_mask)
        frontier_coords = set(zip(y_indices, x_indices))

        visited = set()
        clusters = []

        for start_node in frontier_coords:
            if start_node in visited:
                continue

            cluster = []
            queue = deque([start_node])
            visited.add(start_node)

            while queue:
                cy, cx = queue.popleft()
                cluster.append((cy, cx))

                for ny in range(cy - 1, cy + 2):
                    for nx in range(cx - 1, cx + 2):
                        neighbor = (ny, nx)
                        if neighbor in frontier_coords and neighbor not in visited:
                            visited.add(neighbor)
                            queue.append(neighbor)

            if len(cluster) >= self.min_cluster_size:
                clusters.append(cluster)

        return clusters

    def select_best_cluster(self, clusters):
        if not clusters or self.map_info is None:
            return None

        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y
        resolution = self.map_info.resolution
        height, width = self.map_data.shape

        rx, ry = self.robot_x, self.robot_y
        best_target = None
        max_score = -999999.0

        # 0.8 metre emniyet payı = kaç piksel?
        margin_pixels = max(1, int(0.8 / resolution))

        for cluster in clusters:
            # Kümenin merkezini piksel olarak hesapla
            avg_y = sum(p[0] for p in cluster) / len(cluster)
            avg_x = sum(p[1] for p in cluster) / len(cluster)

            # Piksel koordinatını metre (dünya) koordinatına çevir
            wx = origin_x + (avg_x + 0.5) * resolution
            wy = origin_y + (avg_y + 0.5) * resolution

            # Kara liste kontrolü
            in_blacklist = False
            for bx, by in self.blacklist:
                if math.hypot(wx - bx, wy - by) < 1.0:
                    in_blacklist = True
                    break
            if in_blacklist:
                continue

            dist = math.hypot(wx - rx, wy - ry)
            target_angle = math.atan2(wy - ry, wx - rx)

            angle_diff = abs(target_angle - self.robot_yaw)

            while angle_diff > math.pi:
                angle_diff -= 2.0 * math.pi

            angle_diff = abs(angle_diff)

            # Kural 1: Robotun dibine hedef atma (En az 1.2 metre uzaklık)
            if dist < 1.2:
                continue

            # Kural 2: Duvar ve Engel Kontrolü (0.8 metre etrafında engel var mı?)
            my, mx = int(avg_y), int(avg_x)
            near_wall = False
            for dy in range(-margin_pixels, margin_pixels + 1):
                for dx in range(-margin_pixels, margin_pixels + 1):
                    ny, nx = my + dy, mx + dx
                    if 0 <= nx < width and 0 <= ny < height:
                        if self.map_data[ny, nx] > 50:  # Engel/Duvar hücresi
                            near_wall = True
                            break
                if near_wall:
                    break

            if near_wall:
                continue

            # Skorlama: Büyük kümelere öncelik ver, mesafeyi cezalandır
            heading_penalty = angle_diff * 8.0

            score = (
                len(cluster) * 2.0
                - dist * 1.5
                - heading_penalty
            )
            if score > max_score:
                max_score = score
                best_target = (wx, wy)

        return best_target

    def send_goal(self, x, y):
        if self.state != "IDLE":
            return

        self.get_logger().info(f"🎯 Yeni Hedef Gönderiliyor: X={x:.2f}, Y={y:.2f}")
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y

        # --- YÖN HESAPLAMA (ILMEK / KANCA OLUŞUMUNU BİTİREN KISIM) ---
        # Robotun anlık konumundan (self.robot_x, self.robot_y) hedefe olan gidiş açısını buluyoruz
        dx = x - self.robot_x
        dy = y - self.robot_y
        yaw = math.atan2(dy, dx)

        # Yaw açısını Quaternion formatına dönüştür
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        # ------------------------------------------------------------------

        self.current_goal = (x, y)
        self.state = "NAVIGATING"

        self.nav_client.wait_for_server()
        send_goal_future = self.nav_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.goal_response)
    def goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn("Hedef Nav2 tarafından reddedildi! Kara listeye ekleniyor...")
            if self.current_goal:
                self.blacklist.append(self.current_goal)
            self.state = "IDLE"
            return

        handle.get_result_async().add_done_callback(self.goal_result)

    def goal_result(self, future):
        status = future.result().status
        if status != 4:  # Status 4 = SUCCEEDED
            self.get_logger().warn(f"Hedefe ulaşılamadı (Status: {status}). Kara listeye alındı.")
            if self.current_goal:
                self.blacklist.append(self.current_goal)
        else:
            self.get_logger().info("✅ Hedefe başarıyla ulaşıldı!")

        self.state = "IDLE"

    def stop_robot(self):
        twist = Twist()
        


def main(args=None):
    rclpy.init(args=args)
    node = AgresifSahaExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()