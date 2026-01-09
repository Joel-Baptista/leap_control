import rclpy
from std_msgs.msg import Float32, Float32MultiArray
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
import copy

from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory

import time
import numpy as np
import math

MOTOR_MAPPING = {
    "index_pip_flex_joint": 0,
    "index_pip_abb_joint": 1,
    "index_mcp_joint": 2,
    "index_dip_joint": 3,
    "middle_pip_flex_joint": 4,
    "middle_pip_abb_joint": 5,
    "middle_mcp_joint": 6,
    "middle_dip_joint": 7,
    "ring_pip_flex_joint": 8,
    "ring_pip_abb_joint": 9,
    "ring_mcp_joint": 10,
    "ring_dip_joint": 11,
    "thumb_cmc_abb_joint": 12,
    "thumb_cmc_flex_joint": 13,
    "thumb_mcp_joint": 14,
    "thumb_ip_joint": 15,
}


class LeapWebotsDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        self.__motor_0 = self.__robot.getDevice("index_pip_flex_joint")
        self.__motor_1 = self.__robot.getDevice("index_pip_abb_joint")
        self.__motor_2 = self.__robot.getDevice("index_mcp_joint")
        self.__motor_3 = self.__robot.getDevice("index_dip_joint")
        self.__motor_4 = self.__robot.getDevice("middle_pip_flex_joint")
        self.__motor_5 = self.__robot.getDevice("middle_pip_abb_joint")
        self.__motor_6 = self.__robot.getDevice("middle_mcp_joint")
        self.__motor_7 = self.__robot.getDevice("middle_dip_joint")
        self.__motor_8 = self.__robot.getDevice("ring_pip_flex_joint")
        self.__motor_9 = self.__robot.getDevice("ring_pip_abb_joint")
        self.__motor_10 = self.__robot.getDevice("ring_mcp_joint")
        self.__motor_11 = self.__robot.getDevice("ring_dip_joint")
        self.__motor_12 = self.__robot.getDevice("thumb_cmc_abb_joint")
        self.__motor_13 = self.__robot.getDevice("thumb_cmc_flex_joint")
        self.__motor_14 = self.__robot.getDevice("thumb_mcp_joint")
        self.__motor_15 = self.__robot.getDevice("thumb_ip_joint")

        self.__palm_touch_sensor = self.__robot.getDevice("palm_touch_sensor")
        self.__palm_touch_sensor.enable(1)

        self.motor_list = [
            self.__motor_0,
            self.__motor_1,
            self.__motor_2,
            self.__motor_3,
            self.__motor_4,
            self.__motor_5,
            self.__motor_6,
            self.__motor_7,
            self.__motor_8,
            self.__motor_9,
            self.__motor_10,
            self.__motor_11,
            self.__motor_12,
            self.__motor_13,
            self.__motor_14,
            self.__motor_15,
        ]

        # Safety: set a non-zero speed for position control
        for m in self.motor_list:
            if m is None:
                raise RuntimeError(
                    "Missing motor device — check names in the PROTO/WBT."
                )
            # m.setVelocity(1.0)
            m.setPosition(0.0)

        self.joint_states = None
        self.joint_names = None

        self.joint_states_belief = [0.0] * 16 

        self.motor_names = list(MOTOR_MAPPING.keys())

        self.current_goals = None
        self.goals_id = None

        rclpy.init(args=None)
        self.__node = rclpy.create_node("leap_webots_driver")
        # Create subscription on the existing node (NO rclpy.init, NO extra node)
        qos_profile = QoSProfile(depth=10)
        # self.joint_pub = self.__node.create_publisher(
        #     JointState, "joint_states", qos_profile
        # )
        # self.broadcaster = TransformBroadcaster(self.__node, qos=qos_profile)
        # self.timer = self.__node.create_timer(1 / 30, self.update_states)


        time.sleep(4.0)
        self._action_server = ActionServer(
            self.__node,
            FollowJointTrajectory,
            "/hand_controller/follow_joint_trajectory",
            self.execute_callback,
        )

        self.basic_timstep = int(self.__robot.getBasicTimeStep())
        self.__node.get_logger().info("LeapWebotsDriver initialized")

    def update_states(self):

        joint_state = JointState()

        now = self.__node.get_clock().now()
        joint_state.header.stamp = now.to_msg()
        joint_state.name = self.motor_names
        joint_state.position = self.joint_states_belief

        self.joint_pub.publish(joint_state)

    def execute_callback(self, goal_handle: FollowJointTrajectory):

        self.__node.get_logger().info(f"Got Trajectory {goal_handle}")
        st = time.time()
        traj = goal_handle.request.trajectory
        self.joint_names = traj.joint_names

        hz = 1000 / (self.basic_timstep)

        self.last_traj_time = time.time()

        positions = [np.asarray(point.positions) for point in traj.points]
        time_from_start = [
            point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            for point in traj.points
        ]

        interp_positions = []
        if time_from_start[0] != 0:
            positions = [self.joint_states_belief] + positions
            interp_positions.append(self.joint_states_belief)
            time_from_start = [0.0] + time_from_start

        for i in range(0, len(positions) - 1):
            self.__node.get_logger().info(f"i: {i}")
            interp_positions.append(positions[i])
            N = math.floor((time_from_start[i+1] - time_from_start[i]) * hz)
            for j in range(0, N):                
                curr_pos = positions[i] + (positions[i+1] - positions[i]) * (j / N)
                interp_positions.append(curr_pos)

        interp_positions.append(positions[-1])

        self.current_goals = interp_positions
        self.goals_id = 0

        self.__node.get_logger().info(f"self.goals_id: {self.goals_id}")
        self.__node.get_logger().info(f"len(self.current_goals): {len(self.current_goals)}")

        goal_handle.succeed()
        return FollowJointTrajectory.Result()

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)
        # The driver spins the executor; just apply your control here

        # self.__node.get_logger().info(f"{self.__palm_touch_sensor.getValue()}")

        if not (self.current_goals is None or self.goals_id is None):

            names = copy.deepcopy(self.joint_names)

            if self.goals_id < len(self.current_goals):
                joints = self.current_goals[self.goals_id]

                self.__node.get_logger().info(f"Joints: {self.goals_id}")
                self.goals_id += 1
                for i, n in enumerate(names):
                    idx = MOTOR_MAPPING[n]
                    self.motor_list[idx].setPosition(joints[i])
                    self.joint_states_belief[idx] = joints[i]
