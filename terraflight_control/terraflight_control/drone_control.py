import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
import numpy as np


class Drone_Control(Node):
   def __init__(self):
      super().__init__("drone_control")


      # Timer
      self.timer = self.create_timer(0.01, self.timer_callback)

      # TF Listener
      self.tf_buffer = Buffer()
      self.tf_listener = TransformListener(self.tf_buffer, self)
      self.drone_tf = None

      # TF Broadcaster
      self.tf_broadcaster = TransformBroadcaster(self)
      self.drone_tf2 = None
      
   def timer_callback(self):
      self.listen_to_back_apriltag()
      # if self.drone_tf:
      #    self.get_logger().info(f"[1]: {self.drone_tf.transform.translation.x, self.drone_tf.transform.translation.y, self.drone_tf.transform.translation.z}")
      
      self.broadcast_drone()

   def listen_to_back_apriltag(self):
      try:
         self.drone_tf = self.tf_buffer.lookup_transform("chassis", "back_tag", rclpy.time.Time())
      except TransformException as e:
         # self.get_logger().info(f"No transform found: {e}")
         return
      
   def broadcast_drone(self):
      if self.drone_tf:
         self.drone_tf2 = TransformStamped()
         self.drone_tf2.header.stamp = self.get_clock().now().to_msg()
         self.drone_tf2.header.frame_id = "chassis"
         self.drone_tf2.child_frame_id = "drone"

         self.drone_tf2.transform.translation.x = self.drone_tf.transform.translation.z
         self.drone_tf2.transform.translation.y = self.drone_tf.transform.translation.x
         self.drone_tf2.transform.translation.z = -self.drone_tf.transform.translation.y

         self.drone_tf2.transform.rotation.x = self.drone_tf.transform.rotation.x
         self.drone_tf2.transform.rotation.y = self.drone_tf.transform.rotation.y
         self.drone_tf2.transform.rotation.z = self.drone_tf.transform.rotation.z
         self.drone_tf2.transform.rotation.w = self.drone_tf.transform.rotation.w

         self.tf_broadcaster.sendTransform(self.drone_tf2)

         # self.get_logger().info(f"[2]: {self.drone_tf2.transform.translation.x, self.drone_tf2.transform.translation.y, self.drone_tf2.transform.translation.z}")
      else:
         # self.get_logger().info("No drone transform to broadcast")
          return


def drone_control_entry(args=None):
   rclpy.init(args=args)
   node = Drone_Control()
   rclpy.spin(node)
   rclpy.shutdown()   