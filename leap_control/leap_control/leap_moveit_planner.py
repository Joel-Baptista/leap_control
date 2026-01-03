import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory

from dynamixel_sdk import *  # Biblioteca Dynamixel SDK
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray

from rclpy.logging import get_logger

# moveit python library
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters,
)
from moveit.core.kinematic_constraints import construct_joint_constraint


class LeapMoveitPlanner(Node):

    def __init__(self):
        super().__init__("moveit_py")
        self.logger = get_logger("moveit_py.pose_goal")

        # instantiate MoveItPy instance and get planning component
        self.leap = MoveItPy(node_name="moveit_py")
        self.leap_hand = self.leap.get_planning_component("hand")
        self.logger.info("MoveItPy instance created")

        robot_model = self.leap.get_robot_model()
        self.robot_state = RobotState(robot_model)

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
        
        self.subscription = self.create_subscription(Float32MultiArray, '/set_fingers_positions', self.plan_movement, 2000)


    def plan_movement(self, msg):
        # set plan start state to current state
        self.leap_hand.set_start_state_to_current_state()

        # set constraints message

        st = time.time()

        joint_values = {
            key: msg.data[i] for i, key in enumerate(self.ordered_joints)
        }
        self.robot_state.joint_positions = joint_values
        joint_constraint = construct_joint_constraint(
            robot_state=self.robot_state,
            joint_model_group=self.leap.get_robot_model().get_joint_model_group("hand"),
        )
        self.leap_hand.set_goal_state(motion_plan_constraints=[joint_constraint])

        # plan to goal
        self.plan_and_execute(self.leap, self.leap_hand, self.logger, sleep_time=3.0)

        self.logger.debug(f"Time taken for planning: {time.time() - st} seconds")

    def get_rosparam(self, parameter_name, default_value):
        self.declare_parameter(parameter_name, default_value)
        parameter = self.get_parameter(parameter_name).value
        self.get_logger().info(f"Starting finger manager for: {parameter}")

        return parameter

    @staticmethod
    def plan_and_execute(
        robot,
        planning_component,
        logger,
        single_plan_parameters=None,
        multi_plan_parameters=None,
        sleep_time=0.0,
    ):
        """Helper function to plan and execute a motion."""
        # plan to goal
        logger.info("Planning trajectory")
        if multi_plan_parameters is not None:
            plan_result = planning_component.plan(
                multi_plan_parameters=multi_plan_parameters
            )
        elif single_plan_parameters is not None:
            plan_result = planning_component.plan(
                single_plan_parameters=single_plan_parameters
            )
        else:
            plan_result = planning_component.plan()

        # execute the plan
        if plan_result:
            logger.info("Executing plan")
            robot_trajectory = plan_result.trajectory
            robot.execute(robot_trajectory, controllers=[])
        else:
            logger.error("Planning failed")

        time.sleep(sleep_time)




def main(args=None):
    rclpy.init(args=args)
    node = LeapMoveitPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
