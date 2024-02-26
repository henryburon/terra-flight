import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
from std_msgs.msg import String
import modern_robotics as mr
from scipy.spatial.transform import Rotation


def quaternion_from_euler(ai, aj, ak):
        ai /= 2.0
        aj /= 2.0
        ak /= 2.0
        ci = math.cos(ai)
        si = math.sin(ai)
        cj = math.cos(aj)
        sj = math.sin(aj)
        ck = math.cos(ak)
        sk = math.sin(ak)
        cc = ci*ck
        cs = ci*sk
        sc = si*ck
        ss = si*sk

        q = np.empty((4, ))
        q[0] = cj*sc - sj*cs
        q[1] = cj*ss + sj*cc
        q[2] = cj*cs - sj*sc
        q[3] = cj*cc + sj*ss

        return q

class Odometry(Node):
    def __init__(self):
        super().__init__("odometry")

        # Timer
        self.update_rotations_timer = self.create_timer(0.001, self.update_rotations_callback)  # 10 Hz
        self.timer_callback = self.create_timer(0.01, self.timer_callback)  # 100 Hz

        # Subscribers
        self.robot_motion_sub = self.create_subscription(
            String,
            'robot_motion',
            self.robot_motion_callback,
            10)

        # Robot config
        self.robot_config = {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
            "theta": 0.0
        }

        self.robot_motion = "stop"

        # Transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.a_front_left_pin = 2 # blue
        self.b_front_left_pin = 3 # white

        self.a_front_right_pin = 24  # blue
        self.b_front_right_pin = 23  # white

        self.a_back_left_pin = 4 # blue
        self.b_back_left_pin = 17 # white

        self.a_back_right_pin = 27 # blue
        self.b_back_right_pin = 22 # white

        # confirmed, the a pins are correct
        # b pins TBD

        # Set up GPIOs
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.a_front_left_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.b_front_left_pin, GPIO.IN)

        GPIO.setup(self.a_front_right_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.b_front_right_pin, GPIO.IN)

        GPIO.setup(self.a_back_right_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.b_back_right_pin, GPIO.IN)

        GPIO.setup(self.a_back_left_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.b_back_left_pin, GPIO.IN)

         # Register the callback functions
        GPIO.add_event_detect(self.a_front_left_pin, GPIO.RISING, callback=self.front_left_callback)
        GPIO.add_event_detect(self.a_front_right_pin, GPIO.RISING, callback=self.front_right_callback)
        GPIO.add_event_detect(self.a_back_left_pin, GPIO.RISING, callback=self.back_left_callback)
        GPIO.add_event_detect(self.a_back_right_pin, GPIO.RISING, callback=self.back_right_callback)


        self.front_left_state = None
        self.front_left_counter = -1

        self.front_right_state = None
        self.front_right_counter = -1

        self.back_left_state = None
        self.back_left_counter = -1

        self.back_right_state = None
        self.back_right_counter = -1

        # front left, front right, back left, back right
        self.rotation_measurements = [0, 0, 0, 0]
        self.net_rotation = np.array([0, 0, 0, 0])
        self.old_rotations = [0, 0, 0, 0]

        self.log_counter = 0

        self.front_left_direction = 0
        self.front_right_direction = 0
        self.back_left_direction = 0
        self.back_right_direction = 0

    def timer_callback(self):
        # Main timer callback. Updates the robot configuration.
        self.update_robot_config()

    def robot_motion_callback(self, msg):
        self.robot_motion = msg.data

        if self.robot_motion == "forward":
            self.front_left_direction = 1
            self.front_right_direction = 1
            self.back_left_direction = 1
            self.back_right_direction = 1
        elif self.robot_motion == "backward":
            self.front_left_direction = -1
            self.front_right_direction = -1
            self.back_left_direction = -1
            self.back_right_direction = -1
        elif self.robot_motion == "left":
            self.front_left_direction = -1
            self.front_right_direction = 1
            self.back_left_direction = -1
            self.back_right_direction = 1
        elif self.robot_motion == "right":
            self.front_left_direction = 1
            self.front_right_direction = -1
            self.back_left_direction = 1
            self.back_right_direction = -1

    # High-frequency timer to monitor pulses and blindly calculate rotations
    def front_left_callback(self, channel):
        self.front_left_counter += 1
        self.rotation_measurements[0] = self.front_left_counter / 753.2

    def front_right_callback(self, channel):
        self.front_right_counter += 1
        self.rotation_measurements[1] = self.front_right_counter / 753.2

    def back_left_callback(self, channel):
        self.back_left_counter += 1
        self.rotation_measurements[2] = self.back_left_counter / 753.2

    def back_right_callback(self, channel):
        self.back_right_counter += 1
        self.rotation_measurements[3] = self.back_right_counter / 753.2

    def update_rotations_callback(self): # 100 Hz

        # Sensor readings only valid if robot should be moving
        # if self.robot_motion != "stop":

        delta_rotations = np.array(self.rotation_measurements.copy()) - np.array(self.old_rotations)

        delta_rotations[0] = delta_rotations[0] * self.front_left_direction
        delta_rotations[1] = delta_rotations[1] * self.front_right_direction
        delta_rotations[2] = delta_rotations[2] * self.back_left_direction
        delta_rotations[3] = delta_rotations[3] * self.back_right_direction

        self.net_rotation = np.float64(self.net_rotation) + delta_rotations

        self.get_logger().info(f"Net rotation: {self.net_rotation}")

        self.old_rotations = self.rotation_measurements.copy()



    def update_robot_config(self):
        # Update the robot configuration based on the actual rotations of the wheels (do odometry here)
        net_radians = self.net_rotation * 2.0 * np.pi
        r = 0.08575 # may need to decrease due to poor inflation
        D = 0.231775 # half the distance between left and right wheels
        
        # Find twist Vb
        H_pseudo_inv = np.array([[-1/D, 1/D, 1/D, -1/D],
                                [1, 1, 1, 1],
                                [0, 0, 0, 0]])

        reshape_radians = np.array([[net_radians[0]],
                                     [net_radians[1]],
                                     [net_radians[3]], # due to the H_pseudo_inv matrix setup
                                     [net_radians[2]]])

        Vb = (r/4) * np.dot(H_pseudo_inv, reshape_radians)
        Vb6 = np.array([0, 0, Vb[0][0], Vb[1][0], Vb[2][0], 0])

        twist_se3 = mr.VecTose3(Vb6)
        integrated_pose = mr.MatrixExp6(twist_se3)

        position = integrated_pose[0:3, 3]
        rotation_matrix = integrated_pose[0:3, 0:3]

        # Convert the rotation matrix to Euler angles
        theta = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])

        # self.get_logger().info(f"x: {position[0]}, y: {position[1]}, theta: {theta}")

        self.robot_config["x"] = position[0]
        self.robot_config["y"] = position[1]
        self.robot_config["theta"] = theta

        # self.get_logger().info(f"Total rotations: {self.net_rotation}")
        # self.get_logger().info(f"Robot config: {self.robot_config}")

        # update robot configuration
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = self.robot_config["x"]
        t.transform.translation.y = self.robot_config["y"]
        t.transform.translation.z = 0.0

        # q = quaternion_from_euler(0, 0, self.robot_config["theta"]) # in radians

        q = Rotation.from_euler('xyz', [0, 0, self.robot_config["theta"]]).as_quat()

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)


def odometry_entry(args=None):
    rclpy.init(args=args)
    node = Odometry()
    rclpy.spin(node)
    GPIO.cleanup()
    rclpy.shutdown()

if __name__ == "__main__":
    odometry_entry()
