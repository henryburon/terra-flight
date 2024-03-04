import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
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
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
from std_srvs.srv import Empty

class State(Enum):
   ROBOT = auto()
   DRONE = auto()

class DroneState(Enum):
   MANUAL = auto()
   AUTO = auto()

class DroneMovementState(Enum):
   WAITING = auto()
   TAKEOFF = auto()
   LAND = auto()
   RIGHT = auto()
   LEFT = auto()
   FORWARD = auto()
   BACKWARD = auto()
   UP = auto()
   DOWN = auto()
   AUTO_LAND = auto()

class Drone(Node):
   def __init__(self):
      super().__init__("drone")

      self.cbgrp = ReentrantCallbackGroup()

      # Publishers
      self.drone_pub = self.create_publisher(Image, "/drone_camera", 10)
      self.camera_info_pub = self.create_publisher(CameraInfo, '/drone_camera_info', 10)      

      # Subscribers
      self.joy_sub = self.create_subscription(Joy, "/joy", self.joy_callback, 10)

      # Services
      self.takeoff_srv = self.create_service(Empty, "takeoff", self.takeoff_callback)
      self.land_srv = self.create_service(Empty, "land", self.land_callback)
      self.right_srv = self.create_service(Empty, "right", self.right_callback)
      self.left_srv = self.create_service(Empty, "left", self.left_callback)
      self.forward_srv = self.create_service(Empty, "forward", self.forward_callback)
      self.backward_srv = self.create_service(Empty, "backward", self.backward_callback)
      self.up_srv = self.create_service(Empty, "up", self.up_callback)
      self.down_srv = self.create_service(Empty, "down", self.down_callback)
      self.auto_land_srv = self.create_service(Empty, "auto_land", self.auto_land_callback)

      # Clients
      self.takeoff_client = self.create_client(Empty, "takeoff", callback_group=self.cbgrp)
      while not self.takeoff_client.wait_for_service(timeout_sec=1.0):
         self.get_logger().info("Takeoff service not available, waiting again...")

      self.land_client = self.create_client(Empty, "land", callback_group=self.cbgrp)
      while not self.land_client.wait_for_service(timeout_sec=1.0):
         self.get_logger().info("Land service not available, waiting again...")

      self.right_client = self.create_client(Empty, "right", callback_group=self.cbgrp)
      while not self.right_client.wait_for_service(timeout_sec=1.0):
         self.get_logger().info("Right service not available, waiting again...")

      self.left_client = self.create_client(Empty, "left", callback_group=self.cbgrp)
      while not self.left_client.wait_for_service(timeout_sec=1.0):
         self.get_logger().info("Left service not available, waiting again...")

      self.forward_client = self.create_client(Empty, "forward", callback_group=self.cbgrp)
      while not self.forward_client.wait_for_service(timeout_sec=1.0):
         self.get_logger().info("Forward service not available, waiting again...")

      self.backward_client = self.create_client(Empty, "backward", callback_group=self.cbgrp)
      while not self.backward_client.wait_for_service(timeout_sec=1.0):
         self.get_logger().info("Backward service not available, waiting again...")

      self.up_client = self.create_client(Empty, "up", callback_group=self.cbgrp)
      while not self.up_client.wait_for_service(timeout_sec=1.0):
         self.get_logger().info("Up service not available, waiting again...")

      self.down_client = self.create_client(Empty, "down", callback_group=self.cbgrp)
      while not self.down_client.wait_for_service(timeout_sec=1.0):
         self.get_logger().info("Down service not available, waiting again...")

      self.auto_land_client = self.create_client(Empty, "auto_land", callback_group=self.cbgrp)
      while not self.auto_land_client.wait_for_service(timeout_sec=1.0):
         self.get_logger().info("Auto land service not available, waiting again...")

      # Initialize variables for Tello Drone
      self.drone = tello.Tello()
      self.drone.connect()
      self.drone.streamon()
      self.bridge = CvBridge()
      self.image_timestamp = self.get_clock().now().to_msg()

      # TF Listener
      self.tf_buffer = Buffer()
      self.tf_listener = TransformListener(self.tf_buffer, self)
      self.drone_tf = None

      # TF Broadcaster
      self.tf_broadcaster = TransformBroadcaster(self)
      self.drone_tf2 = None

      # Timers
      self.drone_camera_timer = self.create_timer(1/2, self.drone_image_callback)
      # self.drone_camera_info_timer = self.create_timer(1/5, self.drone_camera_info_callback)
      self.tf_timer = self.create_timer(1/100, self.tf_timer_callback)

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
      self.drone_state = DroneState.MANUAL
      self.move_state = DroneMovementState.WAITING

      self.allow_switch_state_flag = True

      # Static transform
      self.tf_top_static_broadcaster = StaticTransformBroadcaster(self)
      self.tf_back_static_broadcaster = StaticTransformBroadcaster(self)

      # self.make_top_static_apriltag_transforms()
      # self.make_back_static_apriltag_transforms()
      self.temp_x = 0.0
      self.temp_y = 0.0
      self.temp_z = 0.0

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
         
         self.image_timestamp = msg_img.header.stamp

         msg = CameraInfo()
         msg.header.stamp = self.image_timestamp # These two messages must maintain the same time stamp, or the synchronization does not work.
         msg.height = self.height
         msg.width = self.width
         msg.distortion_model = self.distortion_model
         msg.d = self.distortion_coefficients
         msg.k = self.camera_matrix
         msg.r = self.rectification_matrix
         msg.p = self.projection_matrix

         self.camera_info_pub.publish(msg)

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

      self.get_logger().info(f"State: {self.state}", once=True)

      if self.state == State.DRONE and self.drone_state == DroneState.MANUAL and self.move_state == DroneMovementState.WAITING:
         if msg.axes[1] == 1 and msg.axes[4] == 1:
            # Takeoff
            future_takeoff = self.takeoff_client.call_async(Empty.Request())
            future_takeoff.add_done_callback(self.future_takeoff_callback)
            self.move_state = DroneMovementState.TAKEOFF
         
         elif msg.axes[1] == -1 and msg.axes[4] == -1:
            # Land
            future_land = self.land_client.call_async(Empty.Request())
            future_land.add_done_callback(self.future_land_callback)
            self.move_state = DroneMovementState.LAND

         elif msg.axes[2] == -1:
            # Rotate left
            future_left = self.left_client.call_async(Empty.Request())
            future_left.add_done_callback(self.future_left_callback)
            self.move_state = DroneMovementState.LEFT

         elif msg.axes[5] == -1:
            # Rotate right
            future_right = self.right_client.call_async(Empty.Request())
            future_right.add_done_callback(self.future_right_callback)
            self.move_state = DroneMovementState.RIGHT

         elif msg.buttons[2] == 1:
            # Move forward
            future_forward = self.forward_client.call_async(Empty.Request())
            future_forward.add_done_callback(self.future_forward_callback)
            self.move_state = DroneMovementState.FORWARD

         elif msg.buttons[0] == 1:
            # Move backward
            future_backward = self.backward_client.call_async(Empty.Request())
            future_backward.add_done_callback(self.future_backward_callback)
            self.move_state = DroneMovementState.BACKWARD

         elif msg.buttons[5] == 1:
            # Move up
            future_up = self.up_client.call_async(Empty.Request())
            future_up.add_done_callback(self.future_up_callback)
            self.move_state = DroneMovementState.UP

         elif msg.buttons[4] == 1:
            # Move down
            future_down = self.down_client.call_async(Empty.Request())
            future_down.add_done_callback(self.future_down_callback)
            self.move_state = DroneMovementState.DOWN

         elif msg.buttons[11] == 1 and msg.buttons[12] == 1:
            # Auto land
            future_auto_land = self.auto_land_client.call_async(Empty.Request())
            future_auto_land.add_done_callback(self.future_auto_land_callback)
            self.move_state = DroneMovementState.AUTO_LAND

   def tf_timer_callback(self):
      self.listen_to_back_apriltag()
      self.broadcast_drone()

   def listen_to_back_apriltag(self):
      try:
         self.drone_tf = self.tf_buffer.lookup_transform("chassis", "back_tag", rclpy.time.Time())
      except TransformException as e:
         # self.get_logger().info(f"No transform found: {e}")
         return
      
   def broadcast_drone(self): # Essentially just the publishing the back_tag transform
      if self.drone_tf:
         self.drone_tf2 = TransformStamped()
         self.drone_tf2.header.stamp = self.get_clock().now().to_msg()
         self.drone_tf2.header.frame_id = "chassis"
         self.drone_tf2.child_frame_id = "drone"

         self.drone_tf2.transform.translation.x = self.drone_tf.transform.translation.z
         self.drone_tf2.transform.translation.y = self.drone_tf.transform.translation.x - 0.25
         self.drone_tf2.transform.translation.z = self.drone_tf.transform.translation.y

         self.drone_tf2.transform.rotation.x = self.drone_tf.transform.rotation.x
         self.drone_tf2.transform.rotation.y = self.drone_tf.transform.rotation.y
         self.drone_tf2.transform.rotation.z = self.drone_tf.transform.rotation.z
         self.drone_tf2.transform.rotation.w = self.drone_tf.transform.rotation.w

         self.tf_broadcaster.sendTransform(self.drone_tf2)

         if (self.temp_x != self.drone_tf2.transform.translation.x or 
            self.temp_y != self.drone_tf2.transform.translation.y or 
            self.temp_z != self.drone_tf2.transform.translation.z):
               self.get_logger().info(f"Drone at: {self.drone_tf2.transform.translation.x, self.drone_tf2.transform.translation.y, self.drone_tf2.transform.translation.z}")

         self.temp_x = self.drone_tf2.transform.translation.x
         self.temp_y = self.drone_tf2.transform.translation.y
         self.temp_z = self.drone_tf2.transform.translation.z



      else:
         # self.get_logger().info("No drone transform to broadcast")
          return
      
   def takeoff_callback(self, request, response):
      if self.state != State.DRONE:
         return response
      self.get_logger().info("Drone is taking off")
      self.drone.takeoff()
      return response
   
   def land_callback(self, request, response):
      if self.state != State.DRONE:
         return response
      self.get_logger().info("Drone is landing")
      self.drone.land()
      return response
   
   def right_callback(self, request, response):
      if self.state != State.DRONE:
         return response
      self.get_logger().info("Drone is rotating right")
      self.drone.rotate_clockwise(45)
      return response
   
   def left_callback(self, request, response):
      if self.state != State.DRONE:
         return response
      self.get_logger().info("Drone is rotating left")
      self.drone.rotate_counter_clockwise(45)
      return response
   
   def forward_callback(self, request, response):
      if self.state != State.DRONE:
         return response
      self.get_logger().info("Drone is moving forward")
      self.drone.move_forward(70)
      return response
   
   def backward_callback(self, request, response):
      if self.state != State.DRONE:
         return response
      self.get_logger().info("Drone is moving backward")
      self.drone.move_back(70)
      return response
   
   def up_callback(self, request, response):
      if self.state != State.DRONE:
         return response
      self.get_logger().info("Drone is moving up")
      self.drone.move_up(50)
      return response
   
   def down_callback(self, request, response):
      if self.state != State.DRONE:
         return response
      self.get_logger().info("Drone is moving down")
      self.drone.move_down(50)
      return response
   
   # This only works when the drone is behind the robot (i.e. can see the back tag)
   def auto_land_callback(self, request, response):
      if self.state != State.DRONE:
         return response
      self.get_logger().info("Drone is auto landing")

      self.drone.move_up(30) # Ensure drone is above platform

      # Move forward to the platform
      x = int(10 + self.drone_tf2.transform.translation.x * 100)
      self.drone.move_left(20) # accounting for the rightward drift (may be specific to my Tello drone, so adjust as needed)
      self.drone.move_forward(x)

      # Move left/right towards the platform
      if self.drone_tf2.transform.translation.x < 0: # When the drone is to the left, the y value is logged as negative (so, move right)
         # Move right
         y = abs(int(self.drone_tf2.transform.translation.y * 100))
         self.drone.move_right(y)
      else:
         # Move left
         y = abs(int(self.drone_tf2.transform.translation.y * 100))
         self.drone.move_left(y)
      
      self.drone.land()
      return response
   
   ##### Future callbacks #####

   def future_takeoff_callback(self, future_takeoff):
      self.get_logger().info(f"{future_takeoff.result()}")
      self.move_state = DroneMovementState.WAITING

   def future_land_callback(self, future_land):
      self.get_logger().info(f"{future_land.result()}")
      self.move_state = DroneMovementState.WAITING

   def future_right_callback(self, future_right):
      self.get_logger().info(f"{future_right.result()}")
      self.move_state = DroneMovementState.WAITING

   def future_left_callback(self, future_left):
      self.get_logger().info(f"{future_left.result()}")
      self.move_state = DroneMovementState.WAITING

   def future_forward_callback(self, future_forward):
      self.get_logger().info(f"{future_forward.result()}")
      self.move_state = DroneMovementState.WAITING

   def future_backward_callback(self, future_backward):
      self.get_logger().info(f"{future_backward.result()}")
      self.move_state = DroneMovementState.WAITING
   
   def future_up_callback(self, future_up):
      self.get_logger().info(f"{future_up.result()}")
      self.move_state = DroneMovementState.WAITING

   def future_down_callback(self, future_down):
      self.get_logger().info(f"{future_down.result()}")
      self.move_state = DroneMovementState.WAITING

   def future_auto_land_callback(self, future_auto_land):
      self.get_logger().info(f"{future_auto_land.result()}")
      self.move_state = DroneMovementState.WAITING


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

def drone_entry(args=None):
   rclpy.init(args=args)
   node = Drone()
   rclpy.spin(node)
   rclpy.shutdown()