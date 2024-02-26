import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String






class Base_Station(Node):
    def __init__(self):
        super().__init__("base_station")

        # Timer
        self.timer = self.create_timer(0.01, self.timer_callback) # 100 Hz

        # Publishers
        self.robot_motion_pub = self.create_publisher(
            String,
            'robot_motion',
            10)

        # Subscribers
        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)

        # Variables
        self.ps3_axes = []
        self.ps3_buttons = []
        self.dpad = [0,0,0,0]





    def timer_callback(self):

        self.send_robot_commands()

    def joy_callback(self, msg):
        # self.get_logger().info("entering joy callback")

        # Collect ps3 controller input values
        self.ps3_axes = msg.axes
        self.ps3_buttons = msg.buttons

        # Convert inputs to variables
        self.map_joy_inputs()

        # self.get_logger().info(f"Test: {self.left_joystick}")

    def map_joy_inputs(self):
        self.left_joystick = [self.ps3_axes[0], # Left/Right
                              self.ps3_axes[1]] # Up/Down
        self.right_joystick = [self.ps3_axes[3],
                               self.ps3_axes[4]]
        self.left_trigger = [self.ps3_axes[2]]
        self.right_trigger = [self.ps3_axes[5]]

        # Directional pad
        self.dpad = [self.ps3_buttons[13], # Top
                     self.ps3_buttons[16], # Right
                     self.ps3_buttons[14], # Bottom
                     self.ps3_buttons[15]] # Left

    def send_robot_commands(self):
        # Convert input directions to string directions and publish
        robot_motion = String()

        if all(value == 0 for value in self.dpad):
            robot_motion.data = "stop"
        elif self.dpad[0] == 1:
            robot_motion.data = "forward"
        elif self.dpad[1] == 1:
            robot_motion.data = "right"
        elif self.dpad[2] == 1:
            robot_motion.data = "backward"
        elif self.dpad[3] == 1:
            robot_motion.data = "left"
        
        self.robot_motion_pub.publish(robot_motion)
        

     

        



def base_station_entry(args=None):
    rclpy.init(args=args)
    node = Base_Station()
    rclpy.spin(node)
    rclpy.shutdown()