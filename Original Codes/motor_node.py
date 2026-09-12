import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import lgpio
import sys

class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')
        
        # Open gpiochip0 (Confirmed by your 'ls /dev/gpiochip*' command)
        try:
            self.chip = lgpio.gpiochip_open(0)
        except Exception as e:
            self.get_logger().error(f"Could not open gpiochip0: {e}")
            sys.exit(1)
        
        # Pinout mapping
        # ENA/ENB are PWM. IN1-IN4 are Direction pins.
        self.pins = {
            'L_PWM': 18, 'L_IN1': 17, 'L_IN2': 27, 
            'R_PWM': 23, 'R_IN1': 22, 'R_IN2': 24
        }
        
        # --- Dead-zone compensation ---
        # MIN_DUTY: the lowest duty (%) that reliably starts your motors FROM REST.
        # Below this they just buzz and stall. TUNE THIS for your motors + battery:
        # lower it until a turn stalls, then nudge it back up a few points.
        self.MIN_DUTY = 45.0
        # DEADBAND: |speed| below this is treated as a true stop (no creeping).
        self.DEADBAND = 0.02
        
        # Claim all pins as outputs
        for pin in self.pins.values():
            lgpio.gpio_claim_output(self.chip, pin)

        # Subscriber for velocity commands
        self.subscription = self.create_subscription(
            Twist, 
            '/cmd_vel', 
            self.listener_callback, 
            10)
            
        self.get_logger().info("Motor Node Ready. Dead-zone floor active.")

    def set_motor(self, pwm_pin, in1, in2, speed):
        # Limit speed between -1.0 and 1.0
        speed = max(min(speed, 1.0), -1.0)

        # True stop: cut PWM and both direction pins low (coast)
        if abs(speed) < self.DEADBAND:
            lgpio.tx_pwm(self.chip, pwm_pin, 500, 0)
            lgpio.gpio_write(self.chip, in1, 0)
            lgpio.gpio_write(self.chip, in2, 0)
            return

        # Map |speed| in (0, 1] onto [MIN_DUTY, 100] so even a small command
        # delivers enough torque to break static friction and actually start.
        duty = self.MIN_DUTY + (100.0 - self.MIN_DUTY) * abs(speed)
        duty = min(duty, 100.0)

        # PWM at 500Hz for better motor startup torque
        lgpio.tx_pwm(self.chip, pwm_pin, 500, duty)
        
        # DIRECTION LOGIC (Flipped 0/1 to fix the 'moving backward' issue)
        if speed > 0:
            lgpio.gpio_write(self.chip, in1, 0)
            lgpio.gpio_write(self.chip, in2, 1)
        else:
            lgpio.gpio_write(self.chip, in1, 1)
            lgpio.gpio_write(self.chip, in2, 0)

    def listener_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z
        
        # "linear" drives both wheels forward; "angular" rotates them oppositely.
        left_speed = linear + angular
        right_speed = linear - angular
        
        self.set_motor(self.pins['L_PWM'], self.pins['L_IN1'], self.pins['L_IN2'], left_speed)
        self.set_motor(self.pins['R_PWM'], self.pins['R_IN1'], self.pins['R_IN2'], right_speed)

def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Emergency Stop on Ctrl+C
        node.set_motor(node.pins['L_PWM'], node.pins['L_IN1'], node.pins['L_IN2'], 0)
        node.set_motor(node.pins['R_PWM'], node.pins['R_IN1'], node.pins['R_IN2'], 0)
        node.get_logger().info("Motors Stopped.")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

