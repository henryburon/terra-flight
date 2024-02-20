import rclpy
from rclpy.node import Node
from djitellopy import tello
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge



class Drone_Camera(Node):
   def __init__(self):
      super().__init__("drone_camera")

      # Publishers
      self.drone_pub = self.create_publisher(Image, "drone_camera", 10)

      # Initialize variables for Tello Drone
      self.drone = tello.Tello()
      self.drone.connect()
      self.drone.streamon()
      self.bridge = CvBridge()

      # Timers
      self.timer = self.create_timer(1/100, self.timer_callback)
      
      # Flags
      self.start_display_flag = 0


   def timer_callback(self):

      # Fetch image from drone
      image = self.drone.get_frame_read()

      # Resize image and make into numpy array. Change fx and fy to increase
      # the image quality at expense of larger message
      resized = cv2.resize(image.frame, (0,0), fx=0.25, fy=0.25, interpolation=cv2.INTER_AREA)

      # Publish image (cite Courtney)
      msg_img = Image()
      msg_img = self.bridge.cv2_to_imgmsg(resized, "rgb8")
      msg_img.header.stamp = self.get_clock().now().to_msg()
      msg_img.header.frame_id = "world"
      # self.drone_pub.publish(msg_img)



def drone_camera_entry(args=None):
   rclpy.init(args=args)
   node = Drone_Camera()
   rclpy.spin(node)
   rclpy.shutdown()