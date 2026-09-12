import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocityCombiner(Node):
    def __init__(self):
        super().__init__('velocity_combiner')
        
        # Latest received commands
        self.linear_cmd = 0.0
        self.angular_cmd = 0.0

        # Subscriptions
        self.dist_sub = self.create_subscription(
            Twist, '/cmd_vel_distance', self.dist_callback, 10)
        self.angle_sub = self.create_subscription(
            Twist, '/cmd_vel_angle', self.angle_callback, 10)
            
        # Publisher to the motor node
        self.twist_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Timer to publish at a steady rate (e.g., 20Hz)
        self.timer = self.create_timer(0.05, self.publish_combined_vel)

    def dist_callback(self, msg):
        # Grab only the linear movement
        self.linear_cmd = msg.linear.x

    def angle_callback(self, msg):
        # Grab only the angular movement
        self.angular_cmd = msg.angular.z

    def publish_combined_vel(self):
        combined_msg = Twist()
        combined_msg.linear.x = self.linear_cmd
        combined_msg.angular.z = self.angular_cmd
        
        self.twist_pub.publish(combined_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VelocityCombiner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
