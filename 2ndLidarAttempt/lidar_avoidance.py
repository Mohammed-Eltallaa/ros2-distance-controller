import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import sys

class ObjectTracker(Node):
    def __init__(self):
        super().__init__('object_tracker')
        
        # Best Effort QoS voor de LiDAR
        ym_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.ym_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.ym_listener_callback,
            ym_qos_profile)
            
        self.ym_publisher = self.create_publisher(Twist, '/cmd_vel_angle', 10)
        
        self.ym_target_angle = 0.0
        self.ym_Kp = 0.5 
        self.ym_max_steer = 0.5
        self.ym_fov_rad = np.radians(30.0) 

        # NIEUW: Onafhankelijke timer die ALTIJD moet draaien
        self.ym_heartbeat_timer = self.create_timer(1.0, self.ym_heartbeat_callback)

        print("----> TERMINAL: Object Tracker is succesvol geïnitialiseerd!", file=sys.stderr, flush=True)

    def ym_heartbeat_callback(self):
        print("----> TERMINAL HEARTBEAT: De ROS 2 Event Loop (spin) draait correct!", file=sys.stderr, flush=True)

    def ym_listener_callback(self, msg):
        ym_ranges = np.array(msg.ranges)
        
        # Maak de array van hoeken aan
        ym_angles = msg.angle_min + (np.arange(len(ym_ranges)) * msg.angle_increment)
        
        # SOFTWAREMATIGE INVERSIE: Voeg pi toe om de 0 graden naar de overkant te flippen!
        ym_angles = ym_angles + np.pi
        
        # Normaliseer de hoeken weer netjes tussen -pi en pi
        ym_angles = np.arctan2(np.sin(ym_angles), np.cos(ym_angles))
        
        # ---- Vanaf hier blijft de rest van je cone-filtering exact hetzelfde ----
        ym_valid_indices = np.where(
            (ym_ranges > msg.range_min) & 
            (ym_ranges < msg.range_max) & 
            (ym_angles >= -self.ym_fov_rad) & 
            (ym_angles <= self.ym_fov_rad)
        )[0]
        
        if len(ym_valid_indices) == 0:
            print("----> DEBUG: Niets gezien in de 30 graden cone vooruit.", file=sys.stderr, flush=True)
            return

        ym_closest_index = ym_valid_indices[np.argmin(ym_ranges[ym_valid_indices])]
        ym_object_angle = ym_angles[ym_closest_index]

        ym_error = ym_object_angle - self.ym_target_angle
        ym_steering_cmd = self.ym_Kp * ym_error
        ym_steering_cmd = np.clip(ym_steering_cmd, -self.ym_max_steer, self.ym_max_steer)
        
        ym_cmd_msg = Twist()
        ym_cmd_msg.angular.z = float(ym_steering_cmd)
        self.ym_publisher.publish(ym_cmd_msg)
        
        print(f"----> TRACKING: Angle: {np.degrees(ym_object_angle):.1f}° | Steer: {ym_steering_cmd:.2f}", file=sys.stderr, flush=True)
def main(args=None):
    rclpy.init(args=args)
    ym_node = ObjectTracker()
    try:
        rclpy.spin(ym_node)
    except KeyboardInterrupt:
        pass
    finally:
        ym_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
