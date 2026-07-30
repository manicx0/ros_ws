# Operator/LLM controlled — set ONLY via set_robot_state service
bool waiting

# BT controlled autonomously — set by BT when goal queue is empty
bool idle

# Already planned
bool emergency_stop
bool avoidance_enabled
bool mission_active

# Current position and velocity (populated from filtered odometry)
float64 position_x
float64 position_y
float64 position_yaw
float64 linear_velocity
float64 angular_velocity
bool odom_valid

# GPS position (from NavSatFix)
float64 gps_latitude
float64 gps_longitude
bool gps_fix_valid
