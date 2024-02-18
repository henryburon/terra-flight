import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import pyrealsense2 as rs
import cv2
import numpy as np
from rclpy.qos import QoSProfile



class Fetch_Camera(Node):
   def __init__(self):
      super().__init__("fetch_camera")

      qos = QoSProfile(depth=50, reliability=QoSProfile.ReliabilityPolicy.RELIABLE)

      # Publishers
      self.camera_pub = self.create_publisher(Image, "robot_camera", qos)

      # Timers
      self.camera_timer = self.create_timer(5, self.publish_image)

      # Initialization

      # Configure depth and color streams
      self.pipeline = rs.pipeline()
      self.config = rs.config()

      # Get device product line for setting a supporting resolution
      self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
      self.pipeline_profile = self.config.resolve(self.pipeline_wrapper)
      self.device = self.pipeline_profile.get_device()
      self.device_product_line = str(self.device.get_info(rs.camera_info.product_line))

      self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

      self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

      self.pipeline.start(self.config)




   # Called at 100 Hz
   def publish_image(self):
      frames = self.pipeline.wait_for_frames()
      color_frame = frames.get_color_frame()
      if not color_frame:
         return
      
      # Convert images to numpy arrays
      color_image = np.asanyarray(color_frame.get_data())

      # Create Image message
      msg = Image()
      msg.header.stamp = self.get_clock().now().to_msg()
      msg.header.frame_id = "color_frame"
      msg.height = color_image.shape[0]
      msg.width = color_image.shape[1]
      msg.encoding = 'bgr8' # or bgr8?
      msg.is_bigendian = False
      msg.step = color_image.shape[1] * color_image.shape[2]
      msg.data = color_image.tobytes()

      self.camera_pub.publish(msg)
      
      



def fetch_camera_entry(args=None):
   rclpy.init(args=args)
   node = Fetch_Camera()
   rclpy.spin(node)
   rclpy.shutdown()