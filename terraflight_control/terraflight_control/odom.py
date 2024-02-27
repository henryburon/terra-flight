import rclpy
from rclpy.node import Node
import serial
from threading import Thread

class Odom(Node):
    def __init__(self):
        super().__init__("odom")
        self.serial_port = "/dev/ttyACM0"  # Adjust as per your system
        self.ser = serial.Serial(self.serial_port, baudrate=9600)
        self.create_timer(0.01, self.run)  # Create a timer to call run() every 0.1 seconds

    def run(self):
        try:
            data = self.ser.readline().decode('utf-8').strip() 
            if data:
                self.get_logger().info(data)

         # Handle exceptions
                
        except Exception as e:
            self.get_logger().error(f"Error: {e}") 

def odom_entry(args=None):
    rclpy.init(args=args)
    node = Odom()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    odom_entry()
