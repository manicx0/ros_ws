import json
from husky_msgs.msg import FleetState


def format_fleet_state(fleet_state: FleetState) -> str:
    """Format fleet state for LLM prompt context.

    Only fields present in RobotState.msg are included.
    has_goal is intentionally excluded — it is internal to the BT blackboard.
    """
    if fleet_state is None:
        return 'No fleet state available yet'

    robots = []
    for i, robot_id in enumerate(fleet_state.robot_ids):
        state = fleet_state.states[i]
        robot_info = {
            'robot_id': robot_id,
            'emergency_stop': state.emergency_stop,
            'waiting': state.waiting,
            'idle': state.idle,
            'mission_active': state.mission_active,
            'avoidance_enabled': state.avoidance_enabled,
            'odom_valid': state.odom_valid,
            'gps_fix_valid': state.gps_fix_valid,
        }
        if state.odom_valid:
            robot_info['position_x'] = round(state.position_x, 3)
            robot_info['position_y'] = round(state.position_y, 3)
            robot_info['position_yaw'] = round(state.position_yaw, 3)
            robot_info['linear_velocity'] = round(state.linear_velocity, 3)
            robot_info['angular_velocity'] = round(state.angular_velocity, 3)
        if state.gps_fix_valid:
            robot_info['gps_latitude'] = round(state.gps_latitude, 6)
            robot_info['gps_longitude'] = round(state.gps_longitude, 6)
        robots.append(robot_info)
    return json.dumps(robots)
