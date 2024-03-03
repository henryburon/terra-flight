import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo



class Drone_Control(Node):
   def __init__(self):
      super().__init__("drone_control")

      # Publisher
      self.camera_info_pub = self.create_publisher(CameraInfo, '/drone_camera_info', 10)

      # Timer
      self.timer = self.create_timer(0.01, self.timer_callback)

      # declare parameters
      self.declare_parameter("image_height")

      # Load parameters

      self.height = self.get_parameter("image_height").value
      


   def timer_callback(self):
      # self.recognize_apriltags()
      # self.get_logger().info(f"Height: {self.height}")

   

   def recognize_apriltags(self):
      pass

      



def drone_control_entry(args=None):
   rclpy.init(args=args)
   node = Drone_Control()
   rclpy.spin(node)
   rclpy.shutdown()   