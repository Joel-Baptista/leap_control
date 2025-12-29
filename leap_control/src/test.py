#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterType
from moveit.planning import MoveItPy

PARAMS = [
    "robot_description",
    "robot_description_semantic",
    "robot_description_kinematics",
    "planning_pipelines",
    "default_planning_pipeline",
    "ompl",  # OMPL pipeline config lives under this namespace
]

def get_params(node, target_node: str, names):
    cli = node.create_client(GetParameters, f"{target_node}/get_parameters")
    if not cli.wait_for_service(timeout_sec=5.0):
        raise RuntimeError(f"Service {target_node}/get_parameters not available")

    req = GetParameters.Request()
    req.names = list(names)
    fut = cli.call_async(req)

    rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    resp = fut.result()
    if resp is None:
        raise RuntimeError("Failed to get parameters (timeout or error)")

    out = {}
    for name, val in zip(req.names, resp.values):
        if val.type == ParameterType.PARAMETER_STRING:
            out[name] = val.string_value
        elif val.type == ParameterType.PARAMETER_STRING_ARRAY:
            out[name] = list(val.string_array_value)
        elif val.type == ParameterType.PARAMETER_BOOL:
            out[name] = bool(val.bool_value)
        elif val.type == ParameterType.PARAMETER_INTEGER:
            out[name] = int(val.integer_value)
        elif val.type == ParameterType.PARAMETER_DOUBLE:
            out[name] = float(val.double_value)
        else:
            # many MoveIt pipeline configs aren't simple scalar param types
            # (they can be nested YAML loaded as parameters), so skip here
            pass

    return out

def main():
    rclpy.init()
    node = Node("leap_moveitpy_bootstrap")
    node.set_parameters([rclpy.parameter.Parameter("use_sim_time",
                                                  rclpy.parameter.Parameter.Type.BOOL,
                                                  True)])

    # Pull key params from the already-running move_group
    params = get_params(node, "/move_group", PARAMS)

    # Build a minimal config dict for MoveItPy
    cfg = dict(params)
    cfg["use_sim_time"] = True

    moveit = MoveItPy(node_name="leap_moveitpy", config_dict=cfg)

    hand = moveit.get_planning_component("hand")  # <-- your group name
    hand.set_start_state_to_current_state()
    hand.set_goal_state(configuration={"index_mcp_joint": 0.2})
    plan = hand.plan()
    if plan:
        hand.execute()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
