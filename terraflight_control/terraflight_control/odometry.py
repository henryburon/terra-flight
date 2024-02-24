import rclpy
from rclpy.node import Node



class Odometry(Node):
   def __init__(self):
      super().__init__("odometry")

      # Timer
      self.timer = self.create_timer(0.01, self.timer_callback) # 100 Hz





   def timer_callback(self):

      self.get_logger().info("fdhsfsdfjs")





def odometry_entry(args=None):
   rclpy.init(args=args)
   node = Odometry()
   rclpy.spin(node)
   rclpy.shutdown()