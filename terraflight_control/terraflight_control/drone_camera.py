import rclpy
from rclpy.node import Node
from djitellopy import tello
from sensor_msgs.msg import Image
from sensor_msgs.msg import Joy
from cv_bridge import CvBridge
import cv2
from enum import Enum, auto

class State(Enum):
   ROBOT = auto()
   DRONE = auto()

class Drone_Camera(Node):
   def __init__(self):
      super().__init__("drone_camera")

      # Publishers
      self.drone_pub = self.create_publisher(Image, "/drone_camera", 10)

      # Subscribers
      self.joy_sub = self.create_subscription(Joy, "/joy", self.joy_callback, 10)

      # Initialize variables for Tello Drone
      self.drone = tello.Tello()
      self.drone.connect()
      self.drone.streamon()
      self.bridge = CvBridge()

      # Timers
      self.drone_camera_timer = self.create_timer(1/100, self.drone_image_callback)

      # Initialize state
      self.state = State.ROBOT

      self.allow_switch_state_flag = True

   def drone_image_callback(self):      

      if self.state == State.DRONE:
         # Fetch image from drone
         image = self.drone.get_frame_read()
         resized = cv2.resize(image.frame, (0,0), fx=0.25, fy=0.25, interpolation=cv2.INTER_AREA)

         # Publish image (cite Courtney)
         msg_img = Image()
         msg_img = self.bridge.cv2_to_imgmsg(resized, "rgb8")
         msg_img.header.stamp = self.get_clock().now().to_msg()
         msg_img.header.frame_id = "world"
         self.drone_pub.publish(msg_img)

   def joy_callback(self, msg):
      if msg.buttons[10] == 1 and self.allow_switch_state_flag == True:
         if self.state == State.ROBOT:
            self.state = State.DRONE
         elif self.state == State.DRONE:
            self.state = State.ROBOT
         self.allow_switch_state_flag = False
      elif msg.buttons[10] == 0:
         self.allow_switch_state_flag = True

      self.get_logger().info(f"State: {self.state}")
      

def drone_camera_entry(args=None):
   rclpy.init(args=args)
   node = Drone_Camera()
   rclpy.spin(node)
   rclpy.shutdown()