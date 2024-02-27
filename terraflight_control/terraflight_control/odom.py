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
               values = data.split()  # Split the string into a list of values
               if len(values) >= 4:  # Check that there are at least 4 values
                  first_value = values[0]
                  second_value = values[1]
                  third_value = values[2]  # Get the third value (index starts from 0)
                  fourth_value = values[3]
                  self.get_logger().info(f"First value: {first_value}")
                  self.get_logger().info(f"Second value: {second_value}")
                  self.get_logger().info(f"Third value: {third_value}")
                  self.get_logger().info(f"Fourth value: {fourth_value}")
      except Exception as e:
         self.get_logger().error(f"Error: {e}") 

def odom_entry(args=None):
    rclpy.init(args=args)
    node = Odom()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    odom_entry()
