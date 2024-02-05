import rclpy
from rclpy.node import Node






class Base_Station(Node):
    def __init__(self):
        super().__init__("base_station")

        self.timer = self.create_timer(0.01, self.timer_callback) # 100 Hz





    def timer_callback(self):
        self.get_logger().info("the base station node is up!")

def base_station_entry(args=None):
    rclpy.init(args=args)
    node = Base_Station()
    rclpy.spin(node)
    rclpy.shutdown()