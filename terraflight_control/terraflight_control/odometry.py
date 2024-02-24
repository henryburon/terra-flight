import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO


class Odometry(Node):
   def __init__(self):
      super().__init__("odometry")

      # Timer
      self.timer = self.create_timer(0.01, self.timer_callback) # 100 Hz

      self.a_front_right = 24 # blue
      self.b_front_right = 23 # white

      # Set up GPIOs
      GPIO.setmode(GPIO.BCM)

      GPIO.setup(self.a_front_right, GPIO.IN)
      GPIO.setup(self.b_front_right, GPIO.IN)



   def timer_callback(self):

      current_state = GPIO.input(self.a_front_right)

      if current_state == GPIO.HIGH:
         self.get_logger().info("It is HIGH")
      elif current_state == GPIO.LOW:
         self.get_logger().info("It is LOW")





def odometry_entry(args=None):
   rclpy.init(args=args)
   node = Odometry()
   rclpy.spin(node)
   rclpy.shutdown()