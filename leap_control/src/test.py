#!/usr/bin/env python3
"""
A script to outline the fundamentals of the moveit_py motion planning API.
"""

import time

# generic ros libraries
import rclpy
from rclpy.logging import get_logger

# moveit python library
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters,
)
from moveit.core.kinematic_constraints import construct_joint_constraint


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


def main():

    ###################################################################
    # MoveItPy Setup
    ###################################################################
    rclpy.init()
    logger = get_logger("moveit_py.pose_goal")

    # instantiate MoveItPy instance and get planning component
    leap = MoveItPy(node_name="moveit_py")
    leap_hand = leap.get_planning_component("hand")
    logger.info("MoveItPy instance created")

    robot_model = leap.get_robot_model()
    robot_state = RobotState(robot_model)

    ###########################################################################
    # Plan 4 - set goal state with constraints
    ###########################################################################

    # set plan start state to current state
    leap_hand.set_start_state_to_current_state()

    # set constraints message

    st = time.time()

    joint_values = {
        "index_pip_flex_joint": 0.0,
        "index_pip_abb_joint": 0.0,
        "index_mcp_joint": 0.0,
        "index_dip_joint": 0.0,
        "middle_pip_flex_joint": 0.0,
        "middle_pip_abb_joint": 0.0,
        "middle_mcp_joint": 0.0,
        "middle_dip_joint": 0.0,
        "ring_pip_flex_joint": 0.0,
        "ring_pip_abb_joint": 0.0,
        "ring_mcp_joint": 0.0,
        "ring_dip_joint": 0.0,
        "thumb_cmc_abb_joint": 0.0,
        "thumb_cmc_flex_joint": 0.0,
        "thumb_mcp_joint": 0.0,
        "thumb_ip_joint": 0.0,
    }
    robot_state.joint_positions = joint_values
    joint_constraint = construct_joint_constraint(
        robot_state=robot_state,
        joint_model_group=leap.get_robot_model().get_joint_model_group("hand"),
    )
    leap_hand.set_goal_state(motion_plan_constraints=[joint_constraint])

    # plan to goal
    plan_and_execute(leap, leap_hand, logger, sleep_time=3.0)

    logger.debug(f"Time taken for planning: {time.time() - st} seconds")
    
if __name__ == "__main__":
    main()