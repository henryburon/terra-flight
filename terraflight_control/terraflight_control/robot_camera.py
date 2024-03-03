import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import pyrealsense2 as rs
import numpy as np
import cv2
from sensor_msgs.msg import Joy
from enum import Enum, auto

class State(Enum):
   ROBOT = auto()
   DRONE = auto()

class Robot_Camera(Node):
   def __init__(self):
      super().__init__("robot_camera")

      # Publishers
      self.camera_pub = self.create_publisher(Image, "/robot_camera", 10)

      # Subscribers
      self.joy_sub = self.create_subscription(Joy, "/joy", self.joy_callback, 10)

      # Timers
      self.camera_timer = self.create_timer(1/3, self.publish_image)

      # Configure depth and color streams
      self.pipeline = rs.pipeline()
      self.config = rs.config()
      self.config.enable_stream(rs.stream.color, 424, 240, rs.format.bgr8, 6)
      self.pipeline.start(self.config)

      # Initialize state
      self.state = State.ROBOT

      self.allow_switch_state_flag = True

   def publish_image(self):

      # log state
      self.get_logger().info(f"State: {self.state}")

      if self.state == State.ROBOT:
         # Fetch image from Intel Realsense
         frames = self.pipeline.wait_for_frames()
         color_frame = frames.get_color_frame()
         if not color_frame:
            return

         color_image = np.asanyarray(color_frame.get_data())
         color_image = cv2.resize(color_image, (260, 160))
         
         # Create Image message
         msg = Image()
         msg.header.stamp = self.get_clock().now().to_msg()
         msg.header.frame_id = "color_frame"
         msg.height = color_image.shape[0]
         msg.width = color_image.shape[1]
         msg.encoding = 'bgr8'
         msg.is_bigendian = False
         msg.step = color_image.shape[1] * color_image.shape[2]
         msg.data = color_image.tobytes()

         self.camera_pub.publish(msg)

   def joy_callback(self, msg):
    if msg.buttons[10] == 1 and self.allow_switch_state_flag == True:
        if self.state == State.ROBOT:
            self.state = State.DRONE
        elif self.state == State.DRONE:
            self.state = State.ROBOT
        self.allow_switch_state_flag = False
    elif msg.buttons[10] == 0:
        self.allow_switch_state_flag = True
      
def robot_camera_entry(args=None):
   rclpy.init(args=args)
   node = Robot_Camera()
   rclpy.spin(node)
   rclpy.shutdown()