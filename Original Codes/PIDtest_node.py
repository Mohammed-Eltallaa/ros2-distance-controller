import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class PIDDistanceController(Node):
    def __init__(self):
        super().__init__('pid_distance_controller')

        # Directly command the motor node gateway
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.ultra_sub = self.create_subscription(Float32, 'distance_filtered', self.ultra_callback, 10)
        self.lidar_sub = self.create_subscription(Float32, '/path_blocked', self.lidar_callback, 10)

        # State memory storage
        self.lidar_blocked = False
        self.is_escaping = False
        self.latest_ultra_dist = 20.0  # Default safe setpoint distance

        # PID tracking configurations
        self.setpoint = 20.0
        self.kp = 0.22   
        self.ki = 0.001  
        self.kd = 0.02   
        self.integral = 0.0
        self.prev_error = 0.0
        self.max_integral = 5.0  
        
        self.prev_time = self.get_clock().now()

        # FIXED: A dedicated execution timer running at exactly 25Hz (every 0.04s)
        # This guarantees a smooth, continuous stream of commands to the motors with no drops.
        self.control_timer = self.create_timer(0.04, self.control_loop)

        self.get_logger().info("Unified Timer-Driven Brain Online. Lag protection active.")

    def lidar_callback(self, msg):
        # Quietly record state in background
        self.lidar_blocked = (msg.data == 1.0)

    def ultra_callback(self, msg):
        # Quietly record state in background
        self.latest_ultra_dist = msg.data

    def control_loop(self):
        # Pull asynchronous snapshots safely
        ultra_dist = self.latest_ultra_dist
        if ultra_dist <= 0.0:
            return

        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9
        self.prev_time = current_time

        # Safeguard for anomalous timing packets
        if dt <= 0.001:
            dt = 0.04

        # --- STATE MACHINE LOGIC ---
        # Trigger escape rotation if Lidar sees a block OR ultrasonic gets dangerously close
        if self.lidar_blocked or ultra_dist < 15.0:
            self.is_escaping = True

        # Recovery Condition: Stop rotating only when BOTH sensors see a completely clear path ahead
        if self.is_escaping:
            if not self.lidar_blocked and ultra_dist > 40.0:
                self.is_escaping = False
                self.integral = 0.0  # Clear out accumulated winding errors
                self.prev_error = 0.0

        twist = Twist()

        if self.is_escaping:
            # Execute a stable high-torque pivot on the spot
            twist.linear.x = 0.0
            twist.angular.z = -0.65  
        else:
            # Path is wide open! Run your standard forward/backward tracking PID loop
            error = self.setpoint - ultra_dist
            self.integral += error * dt
            self.integral = max(min(self.integral, self.max_integral), -self.max_integral)
            derivative = (error - self.prev_error) / dt

            output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
            cmd_speed = max(min(-output, 0.8), -0.8)

            twist.linear.x = cmd_speed
            twist.angular.z = 0.0
            self.prev_error = error

        # ALWAYS publish a command every single loop execution cycle to prevent motor stalling
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = PIDDistanceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

