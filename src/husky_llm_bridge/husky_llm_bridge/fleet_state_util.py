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
        robots.append({
            'robot_id': robot_id,
            'emergency_stop': state.emergency_stop,
            'waiting': state.waiting,
            'idle': state.idle,
            'mission_active': state.mission_active,
            'avoidance_enabled': state.avoidance_enabled,
        })
    return json.dumps(robots)
