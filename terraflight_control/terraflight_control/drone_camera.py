import rclpy
from rclpy.node import Node
from djitellopy import tello
from sensor_msgs.msg import Image
from sensor_msgs.msg import Joy
from cv_bridge import CvBridge
import cv2
from enum import Enum, auto
from sensor_msgs.msg import CameraInfo

class State(Enum):
   ROBOT = auto()
   DRONE = auto()

class Drone_Camera(Node):
   def __init__(self):
      super().__init__("drone_camera")

      # Publishers
      self.drone_pub = self.create_publisher(Image, "/drone_camera", 10)
      self.camera_info_pub = self.create_publisher(CameraInfo, '/drone_camera_info', 10)

      # Subscribers
      self.joy_sub = self.create_subscription(Joy, "/joy", self.joy_callback, 10)

      # Initialize variables for Tello Drone
      self.drone = tello.Tello()
      self.drone.connect()
      self.drone.streamon()
      self.bridge = CvBridge()
      self.image_timestamp = self.get_clock().now().to_msg()

      # Timers
      self.drone_camera_timer = self.create_timer(1/100, self.drone_image_callback)
      self.drone_camera_info_timer = self.create_timer(1/100, self.drone_camera_info_callback)

      # Parameters
      self.declare_parameter("image_height")
      self.declare_parameter("image_width")
      self.declare_parameter("distortion_model")
      self.declare_parameter("distortion_coefficients")
      self.declare_parameter("camera_matrix")
      self.declare_parameter("rectification_matrix")
      self.declare_parameter("projection_matrix")

      # Load parameters
      self.height = self.get_parameter("image_height").value
      self.width = self.get_parameter("image_width").value
      self.distortion_model = self.get_parameter("distortion_model").value
      self.distortion_coefficients = self.get_parameter("distortion_coefficients").value
      self.camera_matrix = self.get_parameter("camera_matrix").value
      self.rectification_matrix = self.get_parameter("rectification_matrix").value
      self.projection_matrix = self.get_parameter("projection_matrix").value

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

         self.image_timestamp = msg_img.header.stamp

   def drone_camera_info_callback(self):
      msg = CameraInfo()
      msg.header.stamp = self.image_timestamp
      msg.height = self.height
      msg.width = self.width
      msg.distortion_model = self.distortion_model
      msg.d = self.distortion_coefficients
      msg.k = self.camera_matrix
      msg.r = self.rectification_matrix
      msg.p = self.projection_matrix

      self.camera_info_pub.publish(msg)

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