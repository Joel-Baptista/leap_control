import rclpy
from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory

from dynamixel_sdk import *  # Biblioteca Dynamixel SDK
from std_msgs.msg import Int32MultiArray, Float32MultiArray
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from rclpy.action import ActionClient

import numpy as np
import math
import yaml
import os

from utils.leap_base import LeapBase

ON_OFF = ["Off", "On"]


class MovementDemo(LeapBase):

    def __init__(self):
        super().__init__("movement_demo")

        self.ordered_joints = [
            "index_pip_flex_joint",
            "index_pip_abb_joint",
            "index_mcp_joint",
            "index_dip_joint",
            "middle_pip_flex_joint",
            "middle_pip_abb_joint",
            "middle_mcp_joint",
            "middle_dip_joint",
            "ring_pip_flex_joint",
            "ring_pip_abb_joint",
            "ring_mcp_joint",
            "ring_dip_joint",
            "thumb_cmc_abb_joint",
            "thumb_cmc_flex_joint",
            "thumb_mcp_joint",
            "thumb_ip_joint",
        ]

        self.traj_joint_names = None
        self.joint_order_map = {name: i for i, name in enumerate(self.ordered_joints)}

        self.use_planner = self.get_rosparam("use_planner", False)

        pkg_share = get_package_share_directory("leap_control")
        positions_yaml_file = os.path.join(pkg_share, "config", "positions.yaml")

        with open(positions_yaml_file, "r") as file:
            positions = yaml.safe_load(file)

        self.positions = dict(positions)
        self.positions_names = list(self.positions.keys())

        self.planner_publisher = self.create_publisher(
            Float32MultiArray, "/set_fingers_positions", 10
        )
        self._action_client = ActionClient(
            self, FollowJointTrajectory, "/hand_controller/follow_joint_trajectory"
        )

        self.get_logger().info(
            f'Waiting for action server ["/hand_controller/follow_joint_trajectory"]...'
        )
        self.planner_avail = self._action_client.wait_for_server(timeout_sec=1)

        if self.planner_avail:
            self.get_logger().info(f"Moveit planner server is available")
        else:
            self.get_logger().info(
                f"Moveit planner server is not available. Defaulting to control without planner"
            )

        while rclpy.ok():
            for i, pos in enumerate(self.positions_names):
                print(f"[{i}]: {pos}")

            if self.planner_avail:
                print(
                    f"[{i+1}]: Toggle On/Off planner ({ON_OFF[int(self.use_planner)]})"
                )

            input_str = input("Input position: ").strip()

            movement_id = int(input_str)

            if 0 <= movement_id < len(self.positions_names):
                joint_names = [
                    pos for pos in self.positions[self.positions_names[movement_id]]
                ]
                positions = [
                    self.positions[self.positions_names[movement_id]][name]
                    for name in joint_names
                ]
                self.send_trajectory(
                    joint_names=joint_names,
                    positions_list=[positions],
                    time_from_start_list=[2.0],
                )

    def send_trajectory(
        self,
        joint_names,
        positions_list,
        time_from_start_list,
        velocities_list=None,
        accelerations_list=None,
    ):
        """
        Parameters
        ----------
        joint_names : list[str]
        positions_list : list[list[float]]
            One list per trajectory point
        time_from_start_list : list[float]
            Seconds for each point
        velocities_list : list[list[float]], optional
        accelerations_list : list[list[float]], optional
        """

        goal_msg = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names

        for i, positions in enumerate(positions_list):
            point = JointTrajectoryPoint()
            point.positions = positions

            if velocities_list is not None:
                point.velocities = velocities_list[i]

            if accelerations_list is not None:
                point.accelerations = accelerations_list[i]

            point.time_from_start = Duration(
                sec=int(time_from_start_list[i]),
                nanosec=int((time_from_start_list[i] % 1.0) * 1e9),
            )

            trajectory.points.append(point)

        goal_msg.trajectory = trajectory

        self.get_logger().info("Sending trajectory goal...")
        send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Trajectory goal rejected.")
            return

        self.get_logger().info("Trajectory goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().debug(
            f"Feedback received: " f"error positions = {feedback.error.positions}"
        )

    def result_callback(self, future):
        result = future.result().result
        status = future.result().status

        if status == 4:  # ABORTED
            self.get_logger().error("Trajectory execution aborted.")
        elif status == 5:  # CANCELED
            self.get_logger().warn("Trajectory execution canceled.")
        else:
            self.get_logger().info("Trajectory execution succeeded.")

        self.get_logger().info(f"Error code: {result.error_code}")


def main(args=None):

    rclpy.init(args=args)
    node = MovementDemo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
