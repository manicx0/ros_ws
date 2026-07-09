# Operator/LLM controlled — set ONLY via set_robot_state service
bool waiting

# BT controlled autonomously — set by BT when goal queue is empty
bool idle

# Already planned
bool emergency_stop
bool avoidance_enabled
bool mission_active
