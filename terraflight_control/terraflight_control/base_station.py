import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy






class Base_Station(Node):
    def __init__(self):
        super().__init__("base_station")

        # Timer
        self.timer = self.create_timer(0.01, self.timer_callback) # 100 Hz

        # Subscribers
        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)
        

        # Variables
        self.ps3_axes = []
        self.ps3_buttons = []





    def timer_callback(self):
        self.get_logger().info("the base station node is up!", once=True)

    def joy_callback(self, msg):
        self.get_logger().info("entering joy callback")
        self.ps3_axes = msg.axes
        self.ps3_buttons = msg.buttons

        # self.get_logger().info(f'My log message {buttons}')

        # test = buttons[1]

        # self.get_logger().info(f'My log message {test}')
        self.map_joy_inputs()
        self.get_logger().info(f"fdjsfdkjsfd {self.left_joystick}")


    def map_joy_inputs(self):
        # Left/Right, Up/Down
        self.left_joystick = [self.ps3_axes[0],
                              self.ps3_axes[1]]
        self.right_joystick = [self.ps3_axes[3],
                               self.ps3_axes[4]]
        self.left_trigger = [self.ps3_axes[2]]
        self.right_trigger = [self.ps3_axes[5]]
        



def base_station_entry(args=None):
    rclpy.init(args=args)
    node = Base_Station()
    rclpy.spin(node)
    rclpy.shutdown()