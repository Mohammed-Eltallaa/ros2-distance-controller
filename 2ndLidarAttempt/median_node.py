import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class DistanceFilterNode(Node):
    def __init__(self):
        super().__init__('distance_filter')
        # Listen to 'distance_raw'
        self.ym_subscription = self.create_subscription(Float32, '/distance_raw', self.filter_callback, 10)
        # Publish on 'distance_filtered'
        self.ym_publisher = self.create_publisher(Float32, '/distance_filtered', 10)
        
        # Filter State
        self.ym_data_buffer = [0.0, 0.0, 0.0]
        self.ym_min_val = 3.0
        self.ym_max_val = 380.0

    def filter_callback(self, msg):
        ym_raw_val = msg.data
        
        # 1. Range Validation
        if ym_raw_val < self.ym_min_val or ym_raw_val > self.ym_max_val:
            out_msg = Float32()
            out_msg.data = 0.0
            self.ym_publisher.publish(out_msg)
            self.get_logger().warn(f'Out of range: {ym_raw_val:.2f} cm')
            return

        # 2. Median Filter (Depth 3)
        self.ym_data_buffer.pop(0) 
        self.ym_data_buffer.append(ym_raw_val) 
        
        sorted_data = sorted(self.ym_data_buffer)
        ym_median_val = sorted_data[1]

        # 3. Publish
        out_msg = Float32()
        out_msg.data = ym_median_val
        self.ym_publisher.publish(out_msg)
        self.get_logger().info(f'Filtered Median: {ym_median_val:.2f} cm')

def main(args=None):
    rclpy.init(args=args)
    ym_node = DistanceFilterNode()
    try:
        rclpy.spin(ym_node)
    except KeyboardInterrupt:
        pass
    finally:
        ym_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
