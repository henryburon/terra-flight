import rclpy
from rclpy.node import Node
from djitellopy import tello
from sensor_msgs.msg import Image
from sensor_msgs.msg import Joy
from cv_bridge import CvBridge
import cv2
from enum import Enum, auto
from sensor_msgs.msg import CameraInfo
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
import numpy as np

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

      # Static transform
      self.tf_top_static_broadcaster = StaticTransformBroadcaster(self)
      self.tf_back_static_broadcaster = StaticTransformBroadcaster(self)

      self.make_top_static_apriltag_transforms()
      self.make_back_static_apriltag_transforms()


   def drone_image_callback(self):      

      if self.state == State.DRONE:
         # Fetch image from drone
         image = self.drone.get_frame_read()
         resized = cv2.resize(image.frame, (0,0), fx=1.0, fy=1.0, interpolation=cv2.INTER_AREA)

         # Publish image
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

      self.get_logger().info(f"State: {self.state}", once=True)

   def make_top_static_apriltag_transforms(self): # 5 is top

      # Static transformation from chassis to the top april tag
      top_tag_transform = TransformStamped()
      top_tag_transform.header.stamp = self.get_clock().now().to_msg()
      top_tag_transform.header.frame_id = "chassis"
      top_tag_transform.child_frame_id = "top_tag" 
      top_tag_transform.transform.translation.x = 0.26
      top_tag_transform.transform.translation.y = 0.0
      top_tag_transform.transform.translation.z = 0.20
      q = quaternion_from_euler(0.0, 0.0, 0.0)
      top_tag_transform.transform.rotation.x = q[0]
      top_tag_transform.transform.rotation.y = q[1]
      top_tag_transform.transform.rotation.z = q[2]
      top_tag_transform.transform.rotation.w = q[3]

      # self.tf_top_static_broadcaster.sendTransform(top_tag_transform)

   def make_back_static_apriltag_transforms(self): # 6 is back

      # Static transformation from chassis to the back april tag
      back_tag_transform = TransformStamped()
      back_tag_transform.header.stamp = self.get_clock().now().to_msg()
      back_tag_transform.header.frame_id = "chassis"
      back_tag_transform.child_frame_id = "back_tag"
      back_tag_transform.transform.translation.x = 0.50
      back_tag_transform.transform.translation.y = 0.0
      back_tag_transform.transform.translation.z = 0.17
      q = quaternion_from_euler(0.0, 0.0, 0.0)
      back_tag_transform.transform.rotation.x = q[0]
      back_tag_transform.transform.rotation.y = q[1]
      back_tag_transform.transform.rotation.z = q[2]
      back_tag_transform.transform.rotation.w = q[3]

      # self.tf_back_static_broadcaster.sendTransform(back_tag_transform)

      # log the position of the back tag


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

def drone_camera_entry(args=None):
   rclpy.init(args=args)
   node = Drone_Camera()
   rclpy.spin(node)
   rclpy.shutdown()