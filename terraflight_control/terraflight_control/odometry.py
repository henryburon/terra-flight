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
import serial
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener



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

        self.mode = "test" # "test" or "encoders"

        self.stop_time = 0.0
        self.forward_time = 0.0
        self.backward_time = 0.0
        self.right_time = 0.0
        self.left_time = 0.0
        self.x_test = 0.0
        self.y_test = 0.0
        self.theta_test = 0.0
        self.actual_x = 0.0
        self.actual_y = 0.0
        self.actual_theta = 0.0
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.previous_movement = "stop"
        self.previous_x = 0.0
        self.previous_y = 0.0
        self.previous_theta = 0.0
        self.flag = False

        self.serial_port = "/dev/ttyACM0"
        self.ser = serial.Serial(self.serial_port, baudrate=9600)

        # TF Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer
        self.timer_callback = self.create_timer(0.01, self.timer_callback)  # 100 Hz
        self.receive_rotations_callback = self.create_timer(0.01, self.receive_rotations_callback)  # 100 Hz
        self.update_robot_config_callback = self.create_timer(0.01, self.update_robot_config_callback)  # 100 Hz
        self.robot_state_timer = self.create_timer(0.01, self.robot_state_callback)  # 100 Hz

        # # Subscribers
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

        # front left, front right, back left, back right
        self.all_rotations = np.array([0, 0, 0, 0])
        self.net_rotation = np.array([0, 0, 0, 0])
        self.old_rotations = np.array([0, 0, 0, 0])

        self.log_counter = 0

        self.front_left_direction = 0
        self.front_right_direction = 0
        self.back_left_direction = 0
        self.back_right_direction = 0

    def receive_rotations_callback(self):
        if self.mode == "encoders":
            try:
                data = self.ser.readline().decode('utf-8').strip()
                if data:
                    self.all_rotations = [float(x) / 8 for x in data.split(" ")] # div by 4 gives it the 753.2 ticks per rotation
                    # self.get_logger().info(f"Rotations: {self.all_rotations}")
            except Exception as e:
                self.get_logger().error(f"Error: {e}")

    # callback that logs the actual motion
    def timer_callback(self):
        if self.mode == "encoders":
            if self.robot_motion != "stop":
                delta_rotations = np.array(self.all_rotations.copy()) - np.array(self.old_rotations)

                delta_rotations[0] = delta_rotations[0] * self.front_left_direction *  0.637
                delta_rotations[1] = delta_rotations[1] * self.front_right_direction * 1.0
                delta_rotations[2] = delta_rotations[2] * self.back_left_direction * 0.828
                delta_rotations[3] = delta_rotations[3] * self.back_right_direction * 0.7269

                self.net_rotation = np.float64(self.net_rotation) + delta_rotations

            self.old_rotations = self.all_rotations.copy()

            self.get_logger().info(f"Net rotation: {self.net_rotation}")

    def robot_motion_callback(self, msg):
        self.robot_motion = msg.data

        if self.robot_motion == "forward":
            self.front_left_direction = 1
            self.front_right_direction = 1
            self.back_left_direction = 1
            self.back_right_direction = 1
            self.forward_time += 0.01
        elif self.robot_motion == "backward":
            self.front_left_direction = -1
            self.front_right_direction = -1
            self.back_left_direction = -1
            self.back_right_direction = -1
            self.backward_time += 0.01
        elif self.robot_motion == "left":
            self.front_left_direction = -1
            self.front_right_direction = 1
            self.back_left_direction = -1
            self.back_right_direction = 1
            self.left_time += 0.01
        elif self.robot_motion == "right":
            self.front_left_direction = 1
            self.front_right_direction = -1
            self.back_left_direction = 1
            self.back_right_direction = -1
            self.right_time += 0.01
        elif self.robot_motion == "stop":
            self.stop_time += 0.01

    def update_robot_config_callback(self):
        if self.mode == "encoders":
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
        
        # elif self.mode == "test":


    def robot_state_callback(self):

        if self.mode == "test":
            
            # in degrees
            self.theta_test = (self.left_time * 75.68) + (-1 * self.right_time * 70.4156)

            # get magnitude and direction
            mag = (self.forward_time * 0.76362) - (self.backward_time * 0.72984)
            radians = math.radians(self.theta_test)

            # calculate displacement
            if self.robot_motion in ["forward", "backward"]:
                self.x_test = mag * math.cos(radians)
                self.y_test = mag * math.sin(radians)

            if self.robot_motion in ["forward", "backward"]:

                self.x_test += self.offset_x
                self.y_test += self.offset_y

                if self.flag == True:  # ADDED
                    self.x_test -= self.offset_x
                    self.y_test -= self.offset_y
                    self.flag = False

                
                self.previous_movement = "forward/backward"

            elif self.robot_motion in ["left", "right"]:

                self.flag = True

                try:
                    robot_transform = self.tf_buffer.lookup_transform("world", "base_footprint", rclpy.time.Time())
                    self.offset_x = robot_transform.transform.translation.x
                    self.offset_y = robot_transform.transform.translation.y

                except TransformException as e:
                    self.get_logger().error(f"Error: {e}")
                    return

                self.previous_movement = "left/right"



            self.get_logger().info(f"x: {self.x_test}, y: {self.y_test}, theta: {self.theta_test}")
            # log offset
            self.get_logger().info(f"Offset x: {self.offset_x}, Offset y: {self.offset_y}")

            q = quaternion_from_euler(0, 0, math.radians(self.theta_test))

            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "world"
            t.child_frame_id = "base_footprint"
            t.transform.translation.x = self.x_test
            t.transform.translation.y = self.y_test
            t.transform.translation.z = 0.0

            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            self.tf_broadcaster.sendTransform(t)


            


def odometry_entry(args=None):
    rclpy.init(args=args)
    node = Odometry()
    rclpy.spin(node)
    rclpy.shutdown()
