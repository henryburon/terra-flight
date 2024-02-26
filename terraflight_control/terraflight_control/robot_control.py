import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO




class Robot_Control(Node):
    def __init__(self):
        super().__init__("robot_control")

        # Timer
        self.timer = self.create_timer(0.01, self.timer_callback) # 100 Hz

        # Subscribers
        self.robot_motion_sub = self.create_subscription(
            String,
            'robot_motion',
            self.robot_motion_callback,
            10)
        
        # Variables
        self.robot_motion = "stop"

        # GPIO setup
        self.front_left = 12
        self.front_right = 13
        self.back_left = 18
        self.back_right = 19

        GPIO.setmode(GPIO.BCM)

        GPIO.setup(self.front_left, GPIO.OUT)
        GPIO.setup(self.front_right, GPIO.OUT)
        GPIO.setup(self.back_left, GPIO.OUT)
        GPIO.setup(self.back_right, GPIO.OUT)

        frequency = 50
        
        self.pwm_front_left = GPIO.PWM(self.front_left, frequency)
        self.pwm_front_right = GPIO.PWM(self.front_right, frequency)
        self.pwm_back_left = GPIO.PWM(self.back_left, frequency)
        self.pwm_back_right = GPIO.PWM(self.back_right, frequency)

        duty_cycle = (1390/20000) * 100

        self.pwm_front_left.start(duty_cycle)
        self.pwm_front_right.start(duty_cycle)
        self.pwm_back_left.start(duty_cycle)
        self.pwm_back_right.start(duty_cycle)


        
    def timer_callback(self):
        self.command_wheels()

    def robot_motion_callback(self, msg):
        self.robot_motion = msg.data

    def command_wheels(self):
        if self.robot_motion == "stop":
            duty_cycle = (1392/20000) * 100

            self.pwm_front_left.start(duty_cycle)
            self.pwm_front_right.start(duty_cycle)
            self.pwm_back_left.start(duty_cycle)
            self.pwm_back_right.start(duty_cycle)

        elif self.robot_motion == "forward":
            right_duty_cycle = (1200/20000) * 100 # changed from 1200 to 1300
            left_duty_cycle = (1850/20000) * 100

            self.pwm_front_left.start(left_duty_cycle)
            self.pwm_front_right.start(right_duty_cycle)
            self.pwm_back_left.start(left_duty_cycle)
            self.pwm_back_right.start(right_duty_cycle)

        elif self.robot_motion == "backward":
            right_duty_cycle = (1550/20000) * 100
            left_duty_cycle = (1200/20000) * 100

            self.pwm_front_left.start(left_duty_cycle)
            self.pwm_front_right.start(right_duty_cycle)
            self.pwm_back_left.start(left_duty_cycle)
            self.pwm_back_right.start(right_duty_cycle)

        elif self.robot_motion == "right":
            duty_cycle = (1550/20000) * 100

            self.pwm_front_left.start(duty_cycle)
            self.pwm_front_right.start(duty_cycle)
            self.pwm_back_left.start(duty_cycle)
            self.pwm_back_right.start(duty_cycle)

        elif self.robot_motion == "left":
            duty_cycle = (1200/20000) * 100

            self.pwm_front_left.start(duty_cycle)
            self.pwm_front_right.start(duty_cycle)
            self.pwm_back_left.start(duty_cycle)
            self.pwm_back_right.start(duty_cycle)

        # self.get_logger().info(f'Now moving: {self.robot_motion}')

        # Next step is to get this working on the raspberry pi




def robot_control_entry(args=None):
    rclpy.init(args=args)
    node = Robot_Control()
    rclpy.spin(node)
    rclpy.shutdown()