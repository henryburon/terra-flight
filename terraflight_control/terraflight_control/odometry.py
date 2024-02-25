import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO

class Odometry(Node):
    def __init__(self):
        super().__init__("odometry")

        # High-frequency timer to monitor pulses
        self.timer = self.create_timer(0.001, self.timer_callback)  # 1000 Hz

        # Timer to log rotations per second every half second
        self.timer2 = self.create_timer(0.5, self.timer_callback2)

        self.a_front_left = 2 # blue
        self.b_front_left = 3 # white

        self.a_front_right = 24  # blue
        self.b_front_right = 23  # white

        self.a_back_left = 4 # blue
        self.b_back_left = 17 # white

        self.a_back_right = 27 # blue
        self.b_back_right = 22 # white

        # Set up GPIOs
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.a_front_right, GPIO.IN)
        GPIO.setup(self.b_front_right, GPIO.IN)

        self.front_right_state = None
        self.counter = 0

    def timer_callback(self):
        current_state = GPIO.input(self.a_front_right)
        if current_state != self.front_right_state:
            self.counter += 1
        self.front_right_state = current_state
        self.get_logger().info(f"Counter: {self.counter}")

    # def timer_callback2(self):  # Log rotations per second every 0.5s
    #     rotations_per_second = self.counter / 28 / 0.5
    #     self.get_logger().info(f"Rotations per sec: {rotations_per_second}")
    #     self.counter = 0  # Reset the counter for the next period

def odometry_entry(args=None):
    rclpy.init(args=args)
    node = Odometry()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    odometry_entry()
