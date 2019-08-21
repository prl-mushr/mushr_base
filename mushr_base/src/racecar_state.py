#!/usr/bin/env python

from threading import Lock

import numpy as np
import rospy
import tf
import tf.transformations
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.srv import GetMap
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped

import utils


"""
Publishes joint and tf information about the racecar
"""


class RacecarState:
    """
    __init__: Initialize params, publishers, subscribers, and timer
    """

    def __init__(self):
        # speed (rpm) = self.SPEED_TO_ERPM_OFFSET + self.SPEED_TO_ERPM_GAIN * speed (m/s)
        self.SPEED_TO_ERPM_OFFSET = float(
            rospy.get_param("/vesc/speed_to_erpm_offset", 0.0)
        )
        self.SPEED_TO_ERPM_GAIN = float(
            rospy.get_param("/vesc/speed_to_erpm_gain", 4614.0)
        )

        # servo angle = self.STEERING_TO_SERVO_OFFSET + self.STEERING_TO_SERVO_GAIN * steering_angle (rad)
        self.STEERING_TO_SERVO_OFFSET = float(
            rospy.get_param("/vesc/steering_angle_to_servo_offset", 0.5304)
        )
        self.STEERING_TO_SERVO_GAIN = float(
            rospy.get_param("/vesc/steering_angle_to_servo_gain", -1.2135)
        )

        # Length of the car
        self.CAR_LENGTH = float(rospy.get_param("/vesc/chassis_length", 0.33))

        # Width of the car
        self.CAR_WIDTH = float(rospy.get_param("/vesc/wheelbase", 0.25))

        # The radius of the car wheel in meters
        self.CAR_WHEEL_RADIUS = 0.0976 / 2.0

        # Rate at which to publish joints and tf
        self.UPDATE_RATE = float(rospy.get_param("~update_rate", 20.0))

        # Speed noise mean is computed as the most recent speed multiplied by this value
        self.SPEED_OFFSET = float(rospy.get_param("~speed_offset", 0.00))

        # Speed noise std dev
        self.SPEED_NOISE = float(rospy.get_param("~speed_noise", 0.0001))

        # Steering angle noise mean is cimputed as the most recent steering angle multiplied by this value
        self.STEERING_ANGLE_OFFSET = float(
            rospy.get_param("~steering_angle_offset", 0.00)
        )

        # Steering angle noise std dev
        self.STEERING_ANGLE_NOISE = float(
            rospy.get_param("~steering_angle_noise", 0.000001)
        )

        # Forward direction noise mean
        self.FORWARD_OFFSET = float(rospy.get_param("~forward_offset", 0.0))

        # Forward direction noise std dev
        self.FORWARD_FIX_NOISE = float(rospy.get_param("~forward_fix_noise", 0.0000001))

        # Additional zero-mean gaussian noise added to forward direction
        # std dev is most recent velocity times this value
        self.FORWARD_SCALE_NOISE = float(rospy.get_param("~forward_scale_noise", 0.001))

        # Side direction noise mean
        self.SIDE_OFFSET = float(rospy.get_param("~side_offset", 0.0))

        # Side direction noise std dev
        self.SIDE_FIX_NOISE = float(rospy.get_param("~side_fix_noise", 0.000001))

        # Additional zero-mean gaussian noise added to side direction
        # std dev is most recent velocity times this value
        self.SIDE_SCALE_NOISE = float(rospy.get_param("~side_scale_noise", 0.001))

        # Theta noise mean
        self.THETA_OFFSET = float(rospy.get_param("~theta_offset", 0.0))

        # Theta noise std dev
        self.THETA_FIX_NOISE = float(rospy.get_param("~theta_fix_noise", 0.000001))

        # Forces the base_footprint tf to stay in bounds, i.e. updates to the odometry
        # that make base_footprint go out of bounds will be ignored.
        # Only set to true if a map is available.
        self.FORCE_IN_BOUNDS = bool(rospy.get_param("~force_in_bounds", False))

        # The map and map params
        self.permissible_region = None
        self.map_info = None

        # Get the map
        if self.FORCE_IN_BOUNDS:
            self.permissible_region, self.map_info = self.get_map()

        # The most recent time stamp
        self.last_stamp = None

        # The most recent speed (m/s)
        self.last_speed = 0.0
        self.last_speed_lock = Lock()

        # The most recent steering angle (rad)
        self.last_steering_angle = 0.0
        self.last_steering_angle_lock = Lock()

        # The most recent transform from odom to base_footprint
        self.cur_odom_to_base_trans = np.array([0, 0], dtype=np.float)
        self.cur_odom_to_base_rot = 0.0
        self.cur_odom_to_base_lock = Lock()

        # The most recent transform from the map to odom
        self.cur_map_to_odom_trans = np.array([0, 0], dtype=np.float)
        self.cur_map_to_odom_rot = 0.0
        self.cur_map_to_odom_lock = Lock()

        # Message used to publish joint values
        self.joint_msg = JointState()
        self.joint_msg.name = [
            "front_left_wheel_throttle",
            "front_right_wheel_throttle",
            "back_left_wheel_throttle",
            "back_right_wheel_throttle",
            "front_left_wheel_steer",
            "front_right_wheel_steer",
        ]
        self.joint_msg.position = [0, 0, 0, 0, 0, 0]
        self.joint_msg.velocity = []
        self.joint_msg.effort = []

        # Publishes joint messages
        self.br = tf.TransformBroadcaster()

        # Duration param controls how often to publish default map to odom tf
        # if no other nodes are publishing it
        self.transformer = tf.TransformListener()

        # Publishes joint values
        self.state_pub = rospy.Publisher("/car_pose", PoseStamped, queue_size=1)

        # Publishes joint values
        self.cur_joints_pub = rospy.Publisher("/joint_states", JointState, queue_size=1)

        # Subscribes to the initial pose of the car
        self.init_pose_sub = rospy.Subscriber(
            "/initialpose", PoseWithCovarianceStamped, self.init_pose_cb, queue_size=1
        )

        # Subscribes to info about the bldc (particularly the speed in rpm)
        self.speed_sub = rospy.Subscriber(
            "/vesc/sensors/core", VescStateStamped, self.speed_cb, queue_size=1
        )

        # Subscribes to the position of the servo arm
        self.servo_sub = rospy.Subscriber(
            "/vesc/sensors/servo_position_command", Float64, self.servo_cb, queue_size=1
        )

        # Timer to updates joints and tf
        self.update_timer = rospy.Timer(
            rospy.Duration.from_sec(1.0 / self.UPDATE_RATE), self.timer_cb
        )

    """
    clip_angle: Clip an angle to be between -pi and pi
      val: Angle in radians
      Returns: Equivalent angle between -pi and pi (rad)
    """

    def clip_angle(self, val):
        while val > np.pi:
            val -= 2 * np.pi

        while val < -np.pi:
            val += 2 * np.pi
        return val

    """
    init_pose_cb: Callback to capture the initial pose of the car
      msg: geometry_msg/PoseStamped containing the initial pose
    """

    def init_pose_cb(self, msg):

        # Get the pose of the car w.r.t the map in meters
        rx_trans = np.array(
            [msg.pose.pose.position.x, msg.pose.pose.position.y], dtype=np.float
        )
        rx_rot = utils.quaternion_to_angle(msg.pose.pose.orientation)

        # Get the pose of the car w.r.t the map in pixels
        if self.map_info is not None:
            map_rx_pose = utils.world_to_map(
                (rx_trans[0], rx_trans[1], rx_rot), self.map_info
            )

        # Update the pose of the car if either bounds checking is not enabled,
        # or bounds checking is enabled but the car is in-bounds
        if (
            self.permissible_region is None
            or self.permissible_region[
                int(map_rx_pose[1] + 0.5), int(map_rx_pose[0] + 0.5)
            ]
            == 1
        ):

            self.cur_odom_to_base_lock.acquire()
            self.cur_map_to_odom_lock.acquire()

            # Compute where the car is w.r.t the odometry frame
            offset_in_map = rx_trans - self.cur_map_to_odom_trans
            self.cur_odom_to_base_trans = np.zeros(2, dtype=np.float)
            self.cur_odom_to_base_trans[0] = offset_in_map[0] * np.cos(
                -self.cur_map_to_odom_rot
            ) - offset_in_map[1] * np.sin(-self.cur_map_to_odom_rot)
            self.cur_odom_to_base_trans[1] = offset_in_map[0] * np.sin(
                -self.cur_map_to_odom_rot
            ) + offset_in_map[1] * np.cos(-self.cur_map_to_odom_rot)
            self.cur_odom_to_base_rot = rx_rot - self.cur_map_to_odom_rot

            self.cur_map_to_odom_lock.release()
            self.cur_odom_to_base_lock.release()

    """
    speed_cb: Callback to capture the speed of the car
      msg: vesc_msgs/VescStateStamped message containing the speed of the car (rpm)
    """

    def speed_cb(self, msg):
        self.last_speed_lock.acquire()
        self.last_speed = (
            msg.state.speed - self.SPEED_TO_ERPM_OFFSET
        ) / self.SPEED_TO_ERPM_GAIN
        self.last_speed_lock.release()

    """
    servo_cb: Callback to capture the steering angle of the car
      msg: std_msgs/Float64 message containing the servo value
    """

    def servo_cb(self, msg):
        self.last_steering_angle_lock.acquire()
        self.last_steering_angle = (
            msg.data - self.STEERING_TO_SERVO_OFFSET
        ) / self.STEERING_TO_SERVO_GAIN
        self.last_steering_angle_lock.release()

    """
    timer_cb: Callback occuring at a rate of self.UPDATE_RATE. Updates the car joint
              angles and tf of the base_footprint w.r.t odom. Will also publish
              the tf between odom and map if it detects that no such tf is already
              being published. Also publishes robot state as a PoseStamped msg.
      event: Information about when this callback occurred
    """

    def timer_cb(self, event):
        now = rospy.Time.now()

        # Publish a tf between map and odom if one does not already exist
        # Otherwise, get the most recent tf between map and odom
        self.cur_map_to_odom_lock.acquire()
        try:
            tmp_trans, tmp_rot = self.transformer.lookupTransform(
                "/odom", "/map", rospy.Time(0)
            )
            self.cur_map_to_odom_trans[0] = tmp_trans[0]
            self.cur_map_to_odom_trans[1] = tmp_trans[1]
            self.cur_map_to_odom_rot = (
                tf.transformations.euler_from_quaternion(tmp_rot)
            )[2]

            if tmp_trans[2] == -0.0001:
                self.br.sendTransform(
                    (
                        self.cur_map_to_odom_trans[0],
                        self.cur_map_to_odom_trans[1],
                        0.0001,
                    ),
                    tf.transformations.quaternion_from_euler(
                        0, 0, self.cur_map_to_odom_rot
                    ),
                    now,
                    "/odom",
                    "/map",
                )

        except Exception:
            self.br.sendTransform(
                (self.cur_map_to_odom_trans[0], self.cur_map_to_odom_trans[1], 0.0001),
                tf.transformations.quaternion_from_euler(
                    0, 0, self.cur_map_to_odom_rot
                ),
                now,
                "/odom",
                "/map",
            )
        self.cur_map_to_odom_lock.release()

        # Get the time since the last update
        if self.last_stamp is None:
            self.last_stamp = now
        dt = (now - self.last_stamp).to_sec()

        # Add noise to the speed
        self.last_speed_lock.acquire()
        v = self.last_speed + np.random.normal(
            loc=self.SPEED_OFFSET * self.last_speed, scale=self.SPEED_NOISE, size=1
        )
        self.last_speed_lock.release()

        # Add noise to the steering angle
        self.last_steering_angle_lock.acquire()
        delta = self.last_steering_angle + np.random.normal(
            loc=self.STEERING_ANGLE_OFFSET * self.last_steering_angle,
            scale=self.STEERING_ANGLE_NOISE,
            size=1,
        )
        self.last_steering_angle_lock.release()

        self.cur_odom_to_base_lock.acquire()

        # Apply kinematic car model to the previous pose
        new_pose = np.array(
            [
                self.cur_odom_to_base_trans[0],
                self.cur_odom_to_base_trans[1],
                self.cur_odom_to_base_rot,
            ],
            dtype=np.float,
        )
        if np.abs(delta) < 1e-2:
            # Changes in x, y, and theta
            dtheta = 0
            dx = v * np.cos(self.cur_odom_to_base_rot) * dt
            dy = v * np.sin(self.cur_odom_to_base_rot) * dt

            # New joint values
            joint_left_throttle = v * dt / self.CAR_WHEEL_RADIUS
            joint_right_throttle = v * dt / self.CAR_WHEEL_RADIUS
            joint_left_steer = 0.0
            joint_right_steer = 0.0

        else:
            # Changes in x, y, and theta
            tan_delta = np.tan(delta)
            dtheta = ((v / self.CAR_LENGTH) * tan_delta) * dt
            dx = (self.CAR_LENGTH / tan_delta) * (
                np.sin(self.cur_odom_to_base_rot + dtheta)
                - np.sin(self.cur_odom_to_base_rot)
            )
            dy = (self.CAR_LENGTH / tan_delta) * (
                -1 * np.cos(self.cur_odom_to_base_rot + dtheta)
                + np.cos(self.cur_odom_to_base_rot)
            )

            # New joint values
            # Applt kinematic car model to compute wheel deltas
            h_val = (self.CAR_LENGTH / tan_delta) - (self.CAR_WIDTH / 2.0)
            joint_outer_throttle = (
                ((self.CAR_WIDTH + h_val) / (0.5 * self.CAR_WIDTH + h_val))
                * v
                * dt
                / self.CAR_WHEEL_RADIUS
            )
            joint_inner_throttle = (
                ((h_val) / (0.5 * self.CAR_WIDTH + h_val))
                * v
                * dt
                / self.CAR_WHEEL_RADIUS
            )
            joint_outer_steer = np.arctan2(self.CAR_LENGTH, self.CAR_WIDTH + h_val)
            joint_inner_steer = np.arctan2(self.CAR_LENGTH, h_val)

            # Assign joint values according to whether we are turning left or right
            if (delta) > 0.0:
                joint_left_throttle = joint_inner_throttle
                joint_right_throttle = joint_outer_throttle
                joint_left_steer = joint_inner_steer
                joint_right_steer = joint_outer_steer

            else:
                joint_left_throttle = joint_outer_throttle
                joint_right_throttle = joint_inner_throttle
                joint_left_steer = joint_outer_steer - np.pi
                joint_right_steer = joint_inner_steer - np.pi

        # Apply kinematic model updates and noise to the new pose
        new_pose[0] += (
            dx
            + np.random.normal(
                loc=self.FORWARD_OFFSET, scale=self.FORWARD_FIX_NOISE, size=1
            )
            + np.random.normal(
                loc=0.0, scale=np.abs(v) * self.FORWARD_SCALE_NOISE, size=1
            )
        )
        new_pose[1] += (
            dy
            + np.random.normal(loc=self.SIDE_OFFSET, scale=self.SIDE_FIX_NOISE, size=1)
            + np.random.normal(loc=0.0, scale=np.abs(v) * self.SIDE_SCALE_NOISE, size=1)
        )
        new_pose[2] += dtheta + np.random.normal(
            loc=self.THETA_OFFSET, scale=self.THETA_FIX_NOISE, size=1
        )

        new_pose[2] = self.clip_angle(new_pose[2])

        # Compute the new pose w.r.t the map in meters
        new_map_pose = np.zeros(3, dtype=np.float)
        new_map_pose[0] = self.cur_map_to_odom_trans[0] + (
            new_pose[0] * np.cos(self.cur_map_to_odom_rot)
            - new_pose[1] * np.sin(self.cur_map_to_odom_rot)
        )
        new_map_pose[1] = self.cur_map_to_odom_trans[1] + (
            new_pose[0] * np.sin(self.cur_map_to_odom_rot)
            + new_pose[1] * np.cos(self.cur_map_to_odom_rot)
        )
        new_map_pose[2] = self.cur_map_to_odom_rot + new_pose[2]

        # Get the new pose w.r.t the map in pixels
        if self.map_info is not None:
            new_map_pose = utils.world_to_map(new_map_pose, self.map_info)

        # Update the pose of the car if either bounds checking is not enabled,
        # or bounds checking is enabled but the car is in-bounds
        new_map_pose_x = int(new_map_pose[0] + 0.5)
        new_map_pose_y = int(new_map_pose[1] + 0.5)
        if self.permissible_region is None or (
            new_map_pose_x >= 0
            and new_map_pose_x < self.permissible_region.shape[1]
            and new_map_pose_y >= 0
            and new_map_pose_y < self.permissible_region.shape[0]
            and self.permissible_region[new_map_pose_y, new_map_pose_x] == 1
        ):
            # Update pose of base_footprint w.r.t odom
            self.cur_odom_to_base_trans[0] = new_pose[0]
            self.cur_odom_to_base_trans[1] = new_pose[1]
            self.cur_odom_to_base_rot = new_pose[2]

            # Update joint values
            self.joint_msg.position[0] += joint_left_throttle
            self.joint_msg.position[1] += joint_right_throttle
            self.joint_msg.position[2] += joint_left_throttle
            self.joint_msg.position[3] += joint_right_throttle
            self.joint_msg.position[4] = joint_left_steer
            self.joint_msg.position[5] = joint_right_steer

            # Clip all joint angles
            for i in range(len(self.joint_msg.position)):
                self.joint_msg.position[i] = self.clip_angle(self.joint_msg.position[i])

        # Publish the tf from odom to base_footprint
        self.br.sendTransform(
            (self.cur_odom_to_base_trans[0], self.cur_odom_to_base_trans[1], 0.0),
            tf.transformations.quaternion_from_euler(0, 0, self.cur_odom_to_base_rot),
            now,
            "base_footprint",
            "odom",
        )

        # Publish the joint states
        self.joint_msg.header.stamp = now
        self.cur_joints_pub.publish(self.joint_msg)

        self.last_stamp = now

        self.cur_odom_to_base_lock.release()

        # Publish current state as a PoseStamped topic
        cur_pose = PoseStamped()
        cur_pose.header.frame_id = "map"
        cur_pose.header.stamp = now
        cur_pose.pose.position.x = (
            self.cur_odom_to_base_trans[0] + self.cur_map_to_odom_trans[0]
        )
        cur_pose.pose.position.y = (
            self.cur_odom_to_base_trans[1] + self.cur_map_to_odom_trans[1]
        )
        cur_pose.pose.position.z = 0.0
        rot = self.cur_odom_to_base_rot + self.cur_map_to_odom_rot
        cur_pose.pose.orientation = utils.angle_to_quaternion(rot)
        self.state_pub.publish(cur_pose)

    """
    get_map: Get the map and map meta data
      Returns: A tuple
                First element is array representing map
                  0 indicates out of bounds, 1 indicates in bounds
                Second element is nav_msgs/MapMetaData message with meta data about the map
    """

    def get_map(self):
        # Use the 'static_map' service (launched by MapServer.launch) to get the map
        map_service_name = rospy.get_param("~static_map", "static_map")
        rospy.wait_for_service(map_service_name)
        map_msg = rospy.ServiceProxy(map_service_name, GetMap)().map
        map_info = map_msg.info  # Save info about map for later use

        # Create numpy array representing map for later use
        array_255 = np.array(map_msg.data).reshape(
            (map_msg.info.height, map_msg.info.width)
        )
        permissible_region = np.zeros_like(array_255, dtype=bool)
        permissible_region[
            array_255 == 0
        ] = 1  # Numpy array of dimension (map_msg.info.height, map_msg.info.width),
        # With values 0: not permissible, 1: permissible
        return permissible_region, map_info


if __name__ == "__main__":
    rospy.init_node("car_pose_node")

    rs = RacecarState()

    rospy.spin()
