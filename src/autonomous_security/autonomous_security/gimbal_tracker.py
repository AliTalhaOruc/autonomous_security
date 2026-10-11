import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time
import math

class SimpleGimbalTracker(Node):
    def __init__(self):
        super().__init__('simple_gimbal_tracker')
        
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/drone_target_data',
            self.target_callback,
            10
        )
        
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/turret_controller/commands',
            10
        )
        
        self.center_x = 320.0
        self.center_y = 240.0
        
        # --- ESKİ SEVİLEN KARARLI AYARLAR ---
        self.k_p = 0.001          
        self.max_step = 0.1       # Eski kararlı değere geri döndü
        self.smooth_factor = 0.80  # Eski yumuşak süzülmeye geri döndü
        
        self.pitch_direction_invert = False 
        
        self.default_yaw = 0.0
        self.default_pitch = -0.2  
        
        self.current_yaw = self.default_yaw
        self.current_pitch = self.default_pitch

        self.last_detection_time = time.time()
        self.timeout_duration = 1.0  

        # --- GÜVENLİK VE KİLİTLENME ŞARTLARI ---
        self.lock_start_time = None      
        self.required_lock_duration = 3.5 
        self.max_firing_distance = 12.0    
        self.min_confidence = 0.80        
        
        self.last_valid_depth = 3.0  

        self.timer = self.create_timer(0.1, self.check_timeout_and_publish)
        self.get_logger().info("Kararlı Modda Gimbal Takipçisi Yeniden Başlatıldı.")

    def target_callback(self, msg):
        if len(msg.data) < 4:
            return

        self.last_detection_time = time.time()
        
        cx = msg.data[0]
        cy = msg.data[1]
        depth = msg.data[2]
        confidence = msg.data[3]

        if depth > 0.1 and not math.isinf(depth) and not math.isnan(depth):
            self.last_valid_depth = depth

        # --- SAKİN VE HAFIZALI KİLİTLENME ---
        if confidence >= self.min_confidence and self.last_valid_depth <= self.max_firing_distance:
            if self.lock_start_time is None:
                self.lock_start_time = time.time()
                self.get_logger().info(f"Drone Kadrajda. Kilit Aranıyor... (Mesafe: {self.last_valid_depth:.2f}m)")
            else:
                lock_duration = time.time() - self.lock_start_time
                if lock_duration >= self.required_lock_duration:
                    self.get_logger().warn("==================================================")
                    self.get_logger().warn(f" HEDEF KİLİTLENDİ! KESİN MESAFE: {self.last_valid_depth:.2f}m")
                    self.get_logger().warn(" KALMAN FILTERA GEÇİŞ YAPILIYOR!")
                    self.get_logger().warn("==================================================")
                    rclpy.shutdown()
                    return
        else:
            self.lock_start_time = None

        # --- KARARLI HİZALAMA MOTORU ---
        error_x = self.center_x - cx  
        
        if self.pitch_direction_invert:
            error_y = self.center_y - cy  
        else:
            error_y = cy - self.center_y  
        
        # Kararlı geniş ölü bölgeler geri geldi
        if abs(error_x) < 20.0:
            target_step_x = 0.0
        else:
            target_step_x = error_x * self.k_p
            target_step_x = max(min(target_step_x, self.max_step), -self.max_step)

        if abs(error_y) < 10.0:
            target_step_y = 0.0
        else:
            target_step_y = error_y * self.k_p
            target_step_y = max(min(target_step_y, self.max_step), -self.max_step)

        self.current_yaw += target_step_x * self.smooth_factor
        self.current_pitch += target_step_y * self.smooth_factor
        
        self.current_yaw = math.atan2(math.sin(self.current_yaw), math.cos(self.current_yaw))  
        self.current_pitch = max(min(self.current_pitch, 1.57), -1.57) 

    def check_timeout_and_publish(self):
        if not rclpy.ok():
            return

        elapsed_time = time.time() - self.last_detection_time
        
        if elapsed_time > self.timeout_duration:
            self.current_yaw += (self.default_yaw - self.current_yaw) * 0.1
            self.current_pitch += (self.default_pitch - self.current_pitch) * 0.1
            self.lock_start_time = None 

        cmd_msg = Float64MultiArray()
        cmd_msg.data = [float(self.current_yaw), float(self.current_pitch)]
        self.publisher.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleGimbalTracker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()

