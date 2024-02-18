import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import pyrealsense2 as rs



class Fetch_Camera(Node):
   def __init__(self):
      super().__init__("fetch_camera")

      # Publishers
      self.camera_pub = self.create_publisher(Image, "robot_camera", 10)

      # Timers
      self.camera_timer = self.create_timer(1/100, self.publish_image)

      # Initialization

      # Configure depth and color streams
      pipeline = rs.pipeline()
      config = rs.config()



   # Called at 100 Hz
   def publish_image(self):
      self.get_logger().info("fsdbfkjsbfksd")
      



def fetch_camera_entry(args=None):
   rclpy.init(args=args)
   node = Fetch_Camera()
   rclpy.spin(node)
   rclpy.shutdown()