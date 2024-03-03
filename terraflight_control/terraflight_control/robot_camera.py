import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

class Robot_Camera(Node):
    def __init__(self):
        super().__init__("robot_camera")

        # Timers
        self.camera_timer = self.create_timer(1/5, self.publish_image_callback)

        # Initialize camera
        self.cap = cv2.VideoCapture(0)

        # Check if the camera opened successfully
        if not self.cap.isOpened():
         print("Error: Could not open external camera.")
         # If the external camera at index 0 didn't open, try other indices
         for i in range(1, 10):
            self.cap = cv2.VideoCapture(i)
            if self.cap.isOpened():
                  print(f"External camera found at index {i}")
                  break
         else:  # No external camera was found
            print("No external camera found. Exiting...")
            exit()

        # Create a publisher
        self.image_pub = self.create_publisher(Image, 'camera/image', 10)

        # Create a CvBridge object
        self.bridge = CvBridge()

    def publish_image_callback(self):
      self.get_logger().info("Publishing image")
      ret, frame = self.cap.read()

      # Resize the image to reduce its quality
      frame = cv2.resize(frame, (320, 240))  # Change the dimensions as needed

      # Convert the image to a ROS image
      try:
         ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
      except CvBridgeError as e:
         print(e)

      # Publish the image
      self.image_pub.publish(ros_image)

def robot_camera_entry(args=None):
    rclpy.init(args=args)
    node = Robot_Camera()
    rclpy.spin(node)
    rclpy.shutdown()