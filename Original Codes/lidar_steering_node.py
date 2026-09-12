import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class LidarSteeringNode(Node):
    def __init__(self):
        super().__init__('lidar_steering_node')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos)
        # Publishes a 1.0 if front is blocked, 0.0 if clear
        self.block_pub = self.create_publisher(Float32, '/path_blocked', 10)

        # 50cm detection threshold for structural blockages
        self.BLOCKED_THRESHOLD = 0.50 

    def lidar_callback(self, msg):
        if not msg.ranges:
            return

        num_readings = len(msg.ranges)
        if num_readings < 360:
            return

        # Sweeping a wide 90-degree front cone window (135 to 225 degrees)
        front_cone = msg.ranges[135:225]
        valid_front = [r for r in front_cone if msg.range_min < r < msg.range_max]
        min_front = min(valid_front) if valid_front else float('inf')

        out_msg = Float32()
        if min_front < self.BLOCKED_THRESHOLD:
            out_msg.data = 1.0  # Path is blocked!
        else:
            out_msg.data = 0.0  # Path is clear!
            
        self.block_pub.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarSteeringNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

