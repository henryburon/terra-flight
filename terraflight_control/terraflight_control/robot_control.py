import rclpy
from rclpy.node import Node
from std_msgs.msg import String




class Robot_Control(Node):
    def __init__(self):
        super().__init__("robot_control")

        # Timer
        self.timer = self.create_timer(0.01, self.timer_callback) # 100 Hz

        # Subscribers
        self.robot_motion_sub = self.create_subscription(
            String,
            'robot_motion',
            self.robot_motion_callback,
            10)
        
        # Variables
        self.robot_motion = "stopped"
        
    def timer_callback(self):
        self.get_logger().info(f'Motion is: {self.robot_motion}')

    def robot_motion_callback(self, msg):
        self.robot_motion = msg.data




def robot_control_entry(args=None):
    rclpy.init(args=args)
    node = Robot_Control()
    rclpy.spin(node)
    rclpy.shutdown()