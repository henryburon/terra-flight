import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
import modern_robotics as mr

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
        self.wheels_timer = self.create_timer(0.001, self.wheels_timer_callback)  # 1000 Hz
        self.validity_timer = self.create_timer(1, self.validity_timer_callback)  # 1 Hz
        self.timer_callback = self.create_timer(0.01, self.timer_callback)  # 100 Hz

        # Robot config
        self.robot_config = {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
            "theta": 0.0
        }

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
        GPIO.setup(self.a_front_left_pin, GPIO.IN)
        GPIO.setup(self.b_front_left_pin, GPIO.IN)

        GPIO.setup(self.a_front_right_pin, GPIO.IN)
        GPIO.setup(self.b_front_right_pin, GPIO.IN)

        GPIO.setup(self.a_back_right_pin, GPIO.IN)
        GPIO.setup(self.b_back_right_pin, GPIO.IN)

        GPIO.setup(self.a_back_left_pin, GPIO.IN)
        GPIO.setup(self.b_back_left_pin, GPIO.IN)

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
        self.total_delta_rotations = np.array([0, 0, 0, 0])
        self.old_rotations = [0, 0, 0, 0]

        self.log_counter = 0

    def timer_callback(self):
        # Main timer callback. Updates the robot configuration.
        self.update_robot_config()

    # High-frequency timer to monitor pulses and blindly calculate rotations
    def wheels_timer_callback(self):

        # Front left wheel
        front_left_state = GPIO.input(self.a_front_left_pin) # check status of pin a
        if front_left_state != self.front_left_state:
            self.front_left_counter += 1
        self.front_left_state = front_left_state

        # Front right wheel
        front_right_state = GPIO.input(self.a_front_right_pin)
        if front_right_state != self.front_right_state:
            self.front_right_counter += 1
        self.front_right_state = front_right_state

        # Back left wheel
        back_left_state = GPIO.input(self.a_back_left_pin)
        if back_left_state != self.back_left_state:
            self.back_left_counter += 1
        self.back_left_state = back_left_state

        # Back right wheel
        back_right_state = GPIO.input(self.a_back_right_pin)
        if back_right_state != self.back_right_state:
            self.back_right_counter += 1
        self.back_right_state = back_right_state

        # Convert to rotations
        self.rotation_measurements[0] = self.front_left_counter / 376.6 # 376.6 pulses per rotation, via data sheet
        self.rotation_measurements[1] = self.front_right_counter / 376.6
        self.rotation_measurements[2] = self.back_left_counter / 376.6
        self.rotation_measurements[3] = self.back_right_counter / 376.6

        # Right now, this only accounts for forward motion. Will need to account for backward motion as well.

        self.log_counter += 1
        if self.log_counter >= 100:
            # self.get_logger().info(f"Rotation measurements: {self.rotation_measurements}")
            self.log_counter = 0  # Reset the counter

    def validity_timer_callback(self): # 10 Hz
        check1 = False
        # Performs a few checks to verify validity of the rotation measurements
        # Decides whether or not to add the measurements to the actual list of rotations
        # delta rotations is the change in rotations from the last time step (0.1s)
        delta_rotations = np.array(self.rotation_measurements.copy()) - np.array(self.old_rotations)
        self.old_rotations = self.rotation_measurements.copy()

        threshold = 0.55
        if np.count_nonzero(delta_rotations > threshold) >= 3:
            check1 = True

        if check1:
            self.total_delta_rotations = np.float64(self.total_delta_rotations) + delta_rotations

        # Should add a check so it only accepts values when I am sending commands


    def update_robot_config(self):
        # Update the robot configuration based on the actual rotations of the wheels (do odometry here)
        total_delta_radians = self.total_delta_rotations * 2.0 * np.pi
        r = 0.08575 # may need to decrease due to poor inflation
        D = 0.231775 # half the distance between left and right wheels
        l = 0.2436 # half the distance between front and back wheels
        
        # Find twist Vb
        H_pseudo_inv = np.array([[-1/D, 1/D, 1/D, -1/D],
                                [1, 1, 1, 1],
                                [0, 0, 0, 0]])

        reshape_radians = np.array([[total_delta_radians[0]],
                                     [total_delta_radians[1]],
                                     [total_delta_radians[3]], # due to the H_pseudo_inv matrix
                                     [total_delta_radians[2]]])

        Vb = (r/4) * np.dot(H_pseudo_inv, reshape_radians)

        # self.get_logger().info(f"Vb: {Vb}")

        # reformat Vb to 6x1
        Vb6 = np.array([0, 0, Vb[0], Vb[1], Vb[2], 0])

        twist_se3 = mr.VecTose3(Vb6)

        integrated_pose = mr.MatrixExp6(twist_se3)

        self.get_logger().info(f"Integrated pose: {integrated_pose}")

        position = integrated_pose[0:3, 3]
    


        self.robot_config["x"] = position[0]
        self.robot_config["y"] = position[1]
    
        # update robot configuration
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = self.robot_config["x"]
        t.transform.translation.y = self.robot_config["y"]
        t.transform.translation.z = 0.0

        q = quaternion_from_euler(0, 0, self.robot_config["theta"]) # in radians
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

if __name__ == "__main__":
    odometry_entry()
