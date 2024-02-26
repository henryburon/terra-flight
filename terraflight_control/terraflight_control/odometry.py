import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO

class Odometry(Node):
    def __init__(self):
        super().__init__("odometry")

        # High-frequency timer to monitor pulses and calculate rotations
        self.wheels_timer = self.create_timer(0.001, self.wheels_timer_callback)  # 1000 Hz

        self.a_front_left_pin = 2 # blue
        self.b_front_left_pin = 3 # white

        self.a_front_right_pin = 24  # blue
        self.b_front_right_pin = 23  # white

        self.a_back_left_pin = 4 # blue
        self.b_back_left_pin = 17 # white

        self.a_back_right_pin = 27 # blue
        self.b_back_right_pin = 22 # white

        # Set up GPIOs
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.a_front_left_pin, GPIO.IN)
        GPIO.setup(self.b_front_left_pin, GPIO.IN)

        GPIO.setup(self.a_front_right_pin, GPIO.IN)
        GPIO.setup(self.b_front_right_pin, GPIO.IN)

        GPIO.setup(self.a_back_right_pin, GPIO.IN)
        GPIO.setup(self.b_back_right_pin, GPIO.IN)
        
        GPIO.setup(self.a_back_left_pin, GPIO.IN)
        GPIO.setup(self.b_back_left_pin, GPIO.IN)


        self.front_left_state = None
        self.front_left_counter = -1

        self.front_right_state = None
        self.front_right_counter = -1

        self.back_left_state = None
        self.back_left_counter = -1

        self.back_right_state = None
        self.back_right_counter = -1

        # front left, front right, back left, back right
        self.rotations = [0, 0, 0, 0]

    def wheels_timer_callback(self):

        # Count (a) pulses for each wheel

        # Front left wheel
        front_left_state = GPIO.input(self.a_front_left_pin) # check status of pin a
        if front_left_state != self.front_left_state:
            self.front_left_counter += 1
        self.front_left_state = front_left_state

        # Front right wheel
        front_right_state = GPIO.input(self.a_front_right_pin)
        if front_right_state != self.front_right_state:
            self.front_right_counter += 1
        self.front_right_state = front_right_state

        # Back left wheel
        back_left_state = GPIO.input(self.a_back_left_pin)
        if back_left_state != self.back_left_state:
            self.back_left_counter += 1
        self.back_left_state = back_left_state

        # Back right wheel
        back_right_state = GPIO.input(self.a_back_right_pin)
        if back_right_state != self.back_right_state:
            self.back_right_counter += 1
        self.back_right_state = back_right_state

        # Convert to rotations
        self.rotations[0] = self.front_left_counter / 376.6
        self.rotations[1] = self.front_right_counter / 376.6
        self.rotations[2] = self.back_left_counter / 376.6
        self.rotations[3] = self.back_right_counter / 376.6

        # Log total rotations
        self.get_logger().info(f"Rotations: {self.rotations}")


def odometry_entry(args=None):
    rclpy.init(args=args)
    node = Odometry()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    odometry_entry()
