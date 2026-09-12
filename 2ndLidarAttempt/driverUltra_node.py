import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time
import lgpio
import sys

class UltrasonicDriverNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_driver')
        self.publisher_ = self.create_publisher(Float32, 'distance_raw', 10)
        self.timer = self.create_timer(0.04, self.measure_distance) # 25Hz
        
        # Open gpiochip0 to share access cleanly with your motor node
        try:
            self.chip = lgpio.gpiochip_open(0)
        except Exception as e:
            self.get_logger().error(f"Could not open gpiochip0: {e}")
            sys.exit(1)
        
        # GPIO Pins Setup (Your chosen pins)
        self.TRIG = 19
        self.ECHO = 26
        
        lgpio.gpio_claim_output(self.chip, self.TRIG)
        lgpio.gpio_claim_input(self.chip, self.ECHO)
        
        # Ensure trigger starts low
        lgpio.gpio_write(self.chip, self.TRIG, 0)
        time.sleep(0.1)
        self.get_logger().info("Ultrasonic LGPIO Driver Started.")

    def measure_distance(self):
        # Trigger pulse (10 microseconds)
        lgpio.gpio_write(self.chip, self.TRIG, 1)
        time.sleep(0.00001)
        lgpio.gpio_write(self.chip, self.TRIG, 0)

        start_time = time.time()
        timeout = start_time + 0.04  # ~40ms max round trip safeguard

        # Wait for echo pulse start
        while lgpio.gpio_read(self.chip, self.ECHO) == 0:
            start_time = time.time()
            if start_time > timeout:
                self.get_logger().error("Sensor Timeout: Echo start not detected")
                return

        # Wait for echo pulse end
        stop_time = time.time()
        while lgpio.gpio_read(self.chip, self.ECHO) == 1:
            stop_time = time.time()
            if stop_time > timeout:
                break # Sensor range exceeded

        duration = stop_time - start_time
        # Speed of sound is 34300 cm/s. Divide by 2 for round-trip path.
        distance = (duration * 34300.0) / 2.0
       
        msg = Float32()
        msg.data = float(distance)
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

