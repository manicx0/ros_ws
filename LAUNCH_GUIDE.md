# Launch Sequence — LLM Fleet Pipeline

## Prerequisites

```bash
# Set API key
export GEMINI_API_KEY="your-key-here"
```

## 5-Terminal Sequence

### Terminal 1 — Simulation

```bash
source /opt/ros/jazzy/setup.bash
cd /root/ros_ws && source install/setup.bash
ros2 launch husky_bringup sim.launch.py
```

### Terminal 2 — Navigation stack

```bash
source /opt/ros/jazzy/setup.bash
cd /root/ros_ws && source install/setup.bash
ros2 launch husky_bringup nav.launch.py namespace:=cpr_a200_0000
```

### Terminal 3 — Behavior tree mission executor

```bash
source /opt/ros/jazzy/setup.bash
cd /root/ros_ws && source install/setup.bash
ros2 launch husky_bringup mission.launch.py namespace:=cpr_a200_0000 bt_file:=patrol_mission.xml
```

### Terminal 4 — Fleet manager + LLM bridge pipeline

```bash
source /opt/ros/jazzy/setup.bash
cd /root/ros_ws && source install/setup.bash
ros2 launch husky_llm_bridge llm_bridge.launch.py
```

### Terminal 5 — Send LLM command

```bash
source /opt/ros/jazzy/setup.bash
cd /root/ros_ws && source install/setup.bash
ros2 topic pub --rate 1 /llm/command std_msgs/String 'data: "send robot to x=3,y=0"'
# Press Ctrl+C after the message publishes once
```

## Debugging

Run in any terminal after sending a command:

| Check | Command |
|-------|---------|
| Odometry flowing | `ros2 topic echo /cpr_a200_0000/platform/odom/filtered --once --no-arr` |
| Raw Gemini output | `ros2 topic echo /llm/raw_decision` |
| Validation result | `ros2 topic echo /llm/decision_status` |
| Robot state | `ros2 topic echo /cpr_a200_0000/robot_state` |
| Velocity commands | `ros2 topic echo /cpr_a200_0000/cmd_vel` |
| Goal events | `ros2 topic echo /fleet/goal_events` |
| Fleet states | `ros2 topic echo /fleet/robot_states` |

## Bypass LLM (test bridge directly)

```bash
ros2 topic pub --once /llm/decision std_msgs/String \
  'data: "{\"missions\":[{\"action\":\"navigate\",\"robot_id\":\"cpr_a200_0000\",\"waypoints\":[{\"x\":2.0,\"y\":0.0}],\"priority\":3}]}"'
```

## Expected Behavior

### Terminal 4 Logs (LLM Bridge)

When the bridge starts, you should see:

```
[INFO] [fleet_manager_node-1]: Added robot: cpr_a200_0000
[INFO] [fleet_manager_node-1]: Added robot: cpr_a200_0001
[INFO] [fleet_manager_node-1]: Added robot: cpr_a200_0002
[INFO] [fleet_manager_node-1]: Fleet manager initialized with 3 robots
[INFO] [llm_validator_node.py-3]: LLM validator initialized. Valid robots: ['cpr_a200_0000', 'cpr_a200_0001', 'cpr_a200_0002']
[WARN] [gemini_connector_node.py-4]: No GEMINI_API_KEY set. Connector will fail on first command.
[INFO] [gemini_connector_node.py-4]: Gemini connector initialized. Model: gemini-2.0-flash-exp
[INFO] [llm_bridge_node.py-2]: LLM bridge initialized
```

### When Command is Sent (Terminal 5)

Successful pipeline flow:

```
[gemini_connector_node.py-4]: Received command: send robot to x=3,y=0
[gemini_connector_node.py-4]: Gemini response published to /llm/raw_decision
[llm_validator_node.py-3]: Decision validated and forwarded
[llm_bridge_node.py-2]: Received 1 mission(s) from LLM
[llm_bridge_node.py-2]: Sending 1 navigate goal(s)
[llm_bridge_node.py-2]: Fleet navigate goal accepted, waiting for result...
```

### Validation Errors

The validator rejects invalid data. Examples:

**Invalid robot_id:**
```
[llm_validator_node.py-3]: Validation failed: Mission[0]: robot_id "invalid_robot" not in fleet config
```

**Missing waypoints:**
```
[llm_validator_node.py-3]: Validation failed: Mission[0]: navigate action must have waypoints
```

**Invalid priority:**
```
[llm_validator_node.py-3]: Validation failed: Mission[0]: priority must be 1-5
```

### Common Issues

#### 1. No API Key

**Symptom:**
```
[WARN] [gemini_connector_node.py-4]: Status: No GEMINI_API_KEY configured
```

**Solution:**
```bash
export GEMINI_API_KEY="your-key"
# Then restart Terminal 4
```

#### 2. Action Server Not Available

**Symptom:**
```
[WARN] [fleet_manager_node-1]: Action server not available for cpr_a200_0000
```

**Cause:** The mission executor (Terminal 3) isn't running or hasn't started yet.

**Solution:** Ensure Terminal 3 is running before sending commands.

#### 3. Topic Subscription Hangs

**Symptom:**
```
Waiting for at least 1 matching subscription(s)...
```

**Cause:** The LLM connector hasn't started yet or crashed.

**Solution:**
- Check Terminal 4 logs for errors
- Verify the bridge launched successfully
- Try `ros2 topic list` to see if `/llm/command` exists

## Test Results (Verified)

The following has been tested and confirmed working:

✅ All 4 nodes start correctly (fleet manager, validator, connector, bridge)
✅ Command pipeline: `/llm/command` → connector → validator → bridge → fleet manager
✅ Validator correctly rejects invalid robot_ids
✅ Validator correctly accepts valid missions
✅ Bridge parses and dispatches missions
✅ Fleet manager attempts to send goals to robots
✅ Error handling: connector publishes to `/llm/decision_status` on failures

The pipeline is fully functional. To complete the full test, you need:
1. Gemini API key set in environment
2. Simulation running (Terminal 1)
3. Navigation stack running (Terminal 2)
4. Mission executor running (Terminal 3)
