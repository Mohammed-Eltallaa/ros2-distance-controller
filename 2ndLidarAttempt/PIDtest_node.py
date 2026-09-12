import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


class PIDDistanceController(Node):

    def __init__(self):
        super().__init__('pid_distance_controller')

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel_distance',
            10
        )

        self.subscription = self.create_subscription(
            Float32,
            '/distance_filtered',
            self.value_callback,
            10
        )

        # Desired distance (cm)
        self.setpoint = 20.0

        # PID gains
        self.kp = 0.22   # Proportional response
        self.ki = 0.001  # Integral response 
        self.kd = 0.02   # Derivative damping

        self.integral = 0.0
        self.prev_error = 0.0
        
        # Anti-Windup Limit: Prevents the integral term from growing infinitely
        # and overriding the Proportional/Derivative reverse commands.
        self.max_integral = 5.0  

        self.prev_distance = None
        
        # Use ROS 2 System Clock instead of Python wall time
        self.prev_time = self.get_clock().now()

        self.get_logger().info("PID Distance Controller Started with Anti-Windup Fix.")

    def value_callback(self, msg):
        distance = msg.data

        # Safeguard against invalid or negative sensor readings
        if distance <= 0.0:
            return

        current_time = self.get_clock().now()
        
        # Calculate dt in seconds from ROS 2 Duration
        dt = (current_time - self.prev_time).nanoseconds / 1e9

        # Prevent division by zero or processing if callbacks arrive too instantly
        if dt <= 0.001:
            return

        # -----------------
        # PID CONTROL
        # -----------------
        error = self.setpoint - distance

        # Accumulate integral and clamp it immediately (Anti-Windup)
        self.integral += error * dt
        self.integral = max(min(self.integral, self.max_integral), -self.max_integral)

        # Calculate derivative
        derivative = (error - self.prev_error) / dt

        # Raw PID Control Output
        output = (
            self.kp * error +
            self.ki * self.integral +
            self.kd * derivative
        )

        # -----------------
        # OBJECT SPEED ESTIMATE
        # -----------------
        object_speed = 0.0
        if self.prev_distance is not None:
            object_speed = (distance - self.prev_distance) / dt

        # -----------------
        # LIMIT SPEED & INVERSION MANAGEMENT
        # -----------------
        max_speed = 1.0 

        # Flip the sign (-output) to match your motor/hardware orientation requirements
        cmd_speed = max(min(-output, max_speed), -max_speed)

        # Publish the movement command
        twist = Twist()
        twist.linear.x = cmd_speed
        self.cmd_pub.publish(twist)

        # Logging output for live debugging and monitoring
        self.get_logger().info(
            f"D={distance:.2f}cm | Err={error:.2f} | Vel={cmd_speed:.2f} | ObjSpeed={object_speed:.2f} cm/s"
        )

        # Save states for the next incoming callback
        self.prev_error = error
        self.prev_distance = distance
        self.prev_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = PIDDistanceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot immediately on shutdown
        stop_twist = Twist()
        node.cmd_pub.publish(stop_twist)
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
