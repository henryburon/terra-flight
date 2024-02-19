import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import pyrealsense2 as rs
import numpy as np

# This node publishes sensor_msgs/Image onto the /robot_camera topic
# Run the node with ros2 run terraflight_control fetch_camera
# The image can be viewed with ros2 run rqt_image_view rqt_image_view
# Ensure you're on the same ROS_DOMAIN_ID
# For some reason, seems to work a little better when plugged into USB 2. It also doesn't recognize the USB 3 as 3, it thinks it's 2.1




class Fetch_Camera(Node):
   def __init__(self):
      super().__init__("fetch_camera")

      # Publishers
      self.camera_pub = self.create_publisher(Image, "robot_camera", 10)

      # Timers
      self.camera_timer = self.create_timer(1/15, self.publish_image)

      # Initialization

      # Configure depth and color streams
      self.pipeline = rs.pipeline()
      self.config = rs.config()

      # Get device product line for setting a supporting resolution
      # self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
      # self.pipeline_profile = self.config.resolve(self.pipeline_wrapper)
      # self.device = self.pipeline_profile.get_device()
      # self.device_product_line = str(self.device.get_info(rs.camera_info.product_line))

      # self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

      # This works on the Raspberry Pi when streaming at 424, 240
      self.config.enable_stream(rs.stream.color, 424, 240, rs.format.bgr8, 15) # (424, 240)

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
      msg.encoding = 'bgr8'
      msg.is_bigendian = False
      msg.step = color_image.shape[1] * color_image.shape[2]
      msg.data = color_image.tobytes()

      self.camera_pub.publish(msg)
      


def fetch_camera_entry(args=None):
   rclpy.init(args=args)
   node = Fetch_Camera()
   rclpy.spin(node)
   rclpy.shutdown()