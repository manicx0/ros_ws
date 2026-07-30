# Mission Lifecycle Manager — Implementation Plan

## Overview

Add async command processing, a mission queue with lifecycle states, and CLI mission
status display. No changes to the C++ fleet manager — the bridge serializes all
missions so only one is active at a time.

## Files to modify (4)

| # | File | Change |
|---|------|--------|
| 1 | `src/husky_llm_bridge/husky_llm_bridge/gemini_connector_node.py` | Async queue + command_id + /llm/command_status |
| 2 | `src/husky_llm_bridge/husky_llm_bridge/deepseek_connector_node.py` | Same async pattern as Gemini |
| 3 | `src/husky_llm_bridge/husky_llm_bridge/ollama_connector_node.py` | Same async pattern |
| 4 | `src/husky_llm_bridge/husky_llm_bridge/llm_bridge_node.py` | Rewrite → mission lifecycle manager with queue |
| 5 | `src/husky_llm_bridge/husky_llm_bridge/cli.py` | Replace waiting_for_response with lifecycle display |

## New topics

| Topic | Type | Publisher | Description |
|-------|------|-----------|-------------|
| `/llm/command_status` | String JSON | Connector nodes | `{"command_id":"...","status":"queued"|"planning"|"done"|"failed"|"skipped","reason":"..."}` |
| `/llm/mission_status` | String JSON | Bridge | `{"command_id":"...","mission_idx":0,"action":"navigate","robot_id":"...","state":"PENDING"|"EXECUTING"|"SUCCEEDED"|"PLANNING_FAILED"|"EXECUTED_FAILED","reason":"...","elapsed":1.5}` |
| `/llm/mission_queue` | String JSON | Bridge | `{"active":{...},"pending":[...],"history":[...]}` |

---

## 1. Gemini Connector → Async (`gemini_connector_node.py`)

### Changes

**New imports** (add at top):
```python
import uuid
import queue
import threading
```

**New members in `__init__`**:
- After `self.status_pub = ...`, add:
  ```python
  self.command_status_pub = self.create_publisher(String, '/llm/command_status', 10)
  ```
- Replace `self.last_command = None` with:
  ```python
  self._last_command_id = None
  self._last_command_text = None
  ```
- Replace `self.create_timer(30.0, self._clear_last_command)` with:
  ```python
  self._command_queue = queue.Queue()
  self._worker_thread = threading.Thread(target=self._process_queue, daemon=True)
  self._worker_thread.start()
  ```

**New helper** (after `_load_fleet_config`):
```python
def _publish_command_status(self, command_id, status, reason=''):
    msg = String()
    msg.data = json.dumps({
        'command_id': command_id,
        'status': status,
        'reason': reason
    })
    self.command_status_pub.publish(msg)
```

**Replace `command_callback`**:
```python
def command_callback(self, msg: String):
    command = msg.data.strip()
    if not command:
        return
    command_id = str(uuid.uuid4())
    if command == getattr(self, '_last_command_text', None):
        if self._last_command_id:
            self._publish_command_status(
                command_id, 'skipped',
                'Duplicate command — already being processed (id: ' + self._last_command_id + ')')
        return
    self.get_logger().info(f'[{command_id[:8]}] Received command: {command}')
    self._last_command_id = command_id
    self._last_command_text = command
    self._publish_command_status(command_id, 'queued')
    self._command_queue.put((command_id, command))
```

**New method** `_process_queue`:
```python
def _process_queue(self):
    while rclpy.ok():
        try:
            command_id, command = self._command_queue.get(timeout=1.0)
        except queue.Empty:
            continue

        self._publish_command_status(command_id, 'planning')
        self.get_logger().info(f'[{command_id[:8]}] Processing command: {command}')

        if self.api_key_from_env:
            api_key = os.environ.get('GEMINI_API_KEY', '')
            if api_key:
                self.api_key = api_key

        if not self.api_key:
            self._publish_command_status(command_id, 'failed', 'No GEMINI_API_KEY configured')
            self._publish_status('No GEMINI_API_KEY configured')
            continue

        fleet_state_json = format_fleet_state(self.latest_fleet_state)
        prompt = self._build_prompt(command, fleet_state_json)

        try:
            response = self._call_gemini(prompt)
            self.raw_decision_pub.publish(self._make_string(response))
            self._publish_command_status(command_id, 'done')
            self.get_logger().info(f'[{command_id[:8]}] Gemini response published')
        except Exception as e:
            reason = f'Gemini API error: {e}'
            self._publish_command_status(command_id, 'failed', reason)
            self._publish_status(reason)
```

**Remove**: `_clear_last_command` method entirely.

---

## 2. DeepSeek Connector → Async (`deepseek_connector_node.py`)

Same changes as Gemini. The `command_callback` and `_process_queue` are identical
except the class name and the API call method name (`_call_deepseek`).

Key differences from Gemini:
- `self._command_queue = queue.Queue()` for typing clarity — use `queue.Queue()` (not `Queue`)
- The `_build_prompt` doesn't have `valid_robots_str` line (Gemini had it, DeepSeek doesn't)
- The `_process_queue` references the same `self.api_key_from_env` pattern but for `DEEPSEEK_API_KEY`

---

## 3. Ollama Connector → Async (`ollama_connector_node.py`)

Same async pattern. The `_process_queue` is identical except:
- Environment variable is `OLLAMA_API_KEY` (or no key for local)
- API call is `_call_ollama` instead of `_call_gemini`/`_call_deepseek`
- 120s timeout instead of 30s

---

## 4. Bridge → Mission Lifecycle Manager (`llm_bridge_node.py`)

### Design

**MissionEntry** (internal data class per mission):
```python
@dataclass
class MissionEntry:
    command_id: str
    mission_idx: int        # index in the missions[] array from the LLM
    action: str
    robot_id: str
    params: dict            # mission dict minus action/robot_id
    state: str              # PENDING, EXECUTING, SUCCEEDED, PLANNING_FAILED, EXECUTED_FAILED
    reason: str             # error or success message
    created_at: float       # time.time()
    started_at: float       # when EXECUTING began
    completed_at: float     # when terminal state reached
```

**State machine**:
```
                 ┌──────────────┐
   /llm/decision │  PENDING     │ ←── enqueued when decision arrives
     ──────────> │  (queued)    │
                 └──────┬───────┘
                        │ _process_queue()
                        v
                 ┌──────────────┐
                 │  EXECUTING   │ ←── dispatch to fleet/set_state/rotate/etc
                 └──────┬───────┘
                   ╱    │    ╲
                  ╱     │     ╲
                 ╱      │      ╲
                ✓       ✓       ✓
         ┌─────────┐ ┌──────┐ ┌──────────┐
         │SUCCEEDED│ │FAILED│ │PLANNING  │
         │         │ │     │ │_FAILED   │ ←── from /llm/decision_status
         └─────────┘ └──────┘ └──────────┘
```

### Subscriptions / Publishers

| Direction | Topic | Type | Notes |
|-----------|-------|------|-------|
| Sub | `/llm/decision` | String | Validated JSON missions |
| Sub | `/llm/decision_status` | String | Validation failures |
| Sub | `/llm/command_status` | String | Connector status (for tracking) |
| Sub | `/fleet/robot_states` | FleetState | Unchanged |
| Sub | `/fleet/goal_events` | GoalEvent | Unchanged |
| Pub | `/llm/mission_status` | String | Per-mission state transitions |
| Pub | `/llm/mission_queue` | String | Full queue snapshot (2 Hz) |

Keep existing per-robot subscriptions (odom) and publishers (rotation_goal,
emergency_stop).

### New members in `__init__`

```python
self._pending_queue = deque()          # deque of MissionEntry
self._active_mission = None            # MissionEntry or None
self._history = []                     # last 10 completed MissionEntry
self._command_status_pub = self.create_publisher(String, '/llm/mission_status', 10)
self._queue_status_pub = self.create_publisher(String, '/llm/mission_queue', 10)
self._command_status_sub = self.create_subscription(
    String, '/llm/command_status', self._command_status_cb, 10)
```

### Key methods

**`decision_callback`** (modified):
- Receive validated JSON from `/llm/decision`
- Create a `MissionEntry` for each mission in the array
- Push each to `_pending_queue` in state `PENDING`
- Call `_process_queue()`
- Publish `mission_queue` snapshot

**`_command_status_cb`** (new):
- Listen for `failed` status from connector
- If a command's LLM call failed, mark all associated pending missions as `PLANNING_FAILED`
- Publish `mission_status` for each failed mission
- Publish `mission_queue` snapshot
- Call `_process_queue()` — but since they were in PENDING and we just failed them,
  `_process_queue` will skip them and move to the next

**`_process_queue`** (new — called when state changes):
```python
def _process_queue(self):
    if self._active_mission is not None:
        return  # already executing one

    # Skip any cancelled/failed pending missions and clean the queue
    while self._pending_queue:
        self._active_mission = self._pending_queue.popleft()
        if self._active_mission.state in ('PLANNING_FAILED',):
            self._archive_mission()
            self._active_mission = None
            continue
        break

    if self._active_mission is None:
        return

    self._active_mission.state = 'EXECUTING'
    self._active_mission.started_at = time.time()
    self._publish_mission_status()

    action = self._active_mission.action
    if action == 'navigate':
        self._execute_navigate(self._active_mission)
    elif action == 'set_state':
        self._execute_set_state(self._active_mission)
    elif action == 'rotate':
        self._execute_rotate(self._active_mission)
    elif action == 'emergency_stop':
        self._execute_emergency_stop(self._active_mission)
    elif action == 'go_home':
        self._execute_go_home(self._active_mission)
    elif action == 'clear_emergency_stop':
        self._execute_clear_emergency_stop(self._active_mission)
```

**`_mission_completed`** (new — called by action callbacks when mission finishes):
```python
def _mission_completed(self, success, reason):
    if self._active_mission is None:
        return
    self._active_mission.state = 'SUCCEEDED' if success else 'EXECUTED_FAILED'
    self._active_mission.reason = reason
    self._active_mission.completed_at = time.time()
    self._publish_mission_status()
    self._archive_mission()
    self._active_mission = None
    self._process_queue()  # try next
```

**Navigate execution** (modified `_send_navigate_missions`):
- Currently sends all navigate missions as one FleetNavigate goal
- **NEW**: Only sends one mission at a time
- `_execute_navigate` creates a `FleetGoal` for one robot, sends it, and the
  existing `_navigate_result_callback` calls `_mission_completed` on success/failure
- The current `_send_navigate_missions` is called for multiple missions from one
  `/llm/decision`. Now each mission goes through the queue individually, so
  we call `_execute_navigate` for each queue entry (which has exactly one mission)

**`_publish_mission_status`** (new):
```python
def _publish_mission_status(self):
    if self._active_mission is None:
        return
    m = self._active_mission
    elapsed = (time.time() - m.started_at) if m.started_at else 0.0
    msg = String()
    msg.data = json.dumps({
        'command_id': m.command_id,
        'mission_idx': m.mission_idx,
        'action': m.action,
        'robot_id': m.robot_id,
        'state': m.state,
        'reason': m.reason,
        'elapsed': round(elapsed, 1)
    })
    self._command_status_pub.publish(msg)
```

**`_publish_queue_status`** (new, on timer or after state change):
```python
def _publish_queue_status(self):
    active = None
    if self._active_mission:
        m = self._active_mission
        elapsed = (time.time() - m.started_at) if m.started_at else 0.0
        active = {
            'command_id': m.command_id,
            'mission_idx': m.mission_idx,
            'action': m.action,
            'robot_id': m.robot_id,
            'state': m.state,
            'elapsed': round(elapsed, 1)
        }
    pending = []
    for m in self._pending_queue:
        pending.append({
            'command_id': m.command_id,
            'mission_idx': m.mission_idx,
            'action': m.action,
            'robot_id': m.robot_id,
            'state': m.state
        })
    history = []
    for m in self._history:
        elapsed = (m.completed_at - m.created_at) if m.completed_at and m.created_at else 0.0
        history.append({
            'command_id': m.command_id,
            'mission_idx': m.mission_idx,
            'action': m.action,
            'robot_id': m.robot_id,
            'state': m.state,
            'reason': m.reason,
            'duration': round(elapsed, 1)
        })
    msg = String()
    msg.data = json.dumps({'active': active, 'pending': pending, 'history': history})
    self._queue_status_pub.publish(msg)
```

### Modify `_send_navigate_missions`

Rename to `_execute_navigate(self, mission: MissionEntry)` — receive a single
mission entry, not a list. Extract `robot_id` and params from `mission.params`.

```python
def _execute_navigate(self, mission: MissionEntry):
    robot_id = mission.robot_id
    params = mission.params

    relative = params.get('relative')
    if relative:
        resolved = self._resolve_relative(mission)
    else:
        resolved = self._resolve_waypoint(params)

    if resolved is None:
        self.get_logger().error(f'Could not resolve waypoint for {robot_id}')
        self._mission_completed(False, f'Could not resolve waypoint')
        return

    x, y, yaw = resolved
    fleet_goal = FleetGoal()
    fleet_goal.robot_id = robot_id

    pose = PoseStamped()
    pose.header.frame_id = 'odom'
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0
    if yaw is not None:
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
    else:
        pose.pose.orientation.w = 1.0
    fleet_goal.target_pose = pose

    goal = FleetNavigate.Goal()
    goal.goals.append(fleet_goal)

    if not self.fleet_navigate_client.wait_for_server(timeout_sec=5.0):
        self._mission_completed(False, 'Fleet navigate action server not available')
        return

    future = self.fleet_navigate_client.send_goal_async(goal)
    future.add_done_callback(self._navigate_goal_response_callback)
```

**Modify `_navigate_goal_response_callback`**:
```python
def _navigate_goal_response_callback(self, future):
    goal_handle = future.result()
    if not goal_handle.accepted:
        self._mission_completed(False, 'Fleet navigate goal rejected')
        return
    self.get_logger().info('Fleet navigate goal accepted')
    result_future = goal_handle.get_result_async()
    result_future.add_done_callback(self._navigate_result_callback)
```

**Modify `_navigate_result_callback`**:
```python
def _navigate_result_callback(self, future):
    result = future.result()
    success = all(r.success for r in result.result.results)
    self._mission_completed(success, 'Navigation complete' if success else result.result.results[0].message if result.result.results else 'Unknown error')
```

### Modify `_resolve_relative` — accept MissionEntry

Change signature from `(self, mission)` to `(self, mission_entry: MissionEntry)`
and use `mission_entry.params` and `mission_entry.robot_id` instead of direct dict access.

Similarly, `_resolve_waypoint` should accept `params` dict instead of full mission.

### Other action executors (`_execute_set_state`, `_execute_rotate`, etc.)

Each follows the same pattern:
```python
def _execute_set_state(self, mission: MissionEntry):
    ...
    # if success:
    self._mission_completed(True, 'State updated')
    # if failure:
    self._mission_completed(False, 'Set state failed: ...')
```

### Decision status handling

Add handler for `/llm/decision_status`:
```python
def _decision_status_callback(self, msg: String):
    try:
        data = json.loads(msg.data)
    except json.JSONDecodeError:
        return
    if data.get('success') is False:
        # Publication failure — no missions were created, so nothing to queue.
        # But we should create a PLANNING_FAILED entry so the CLI sees it.
        # We use a synthetic mission entry with no pending queue entry.
        entry = MissionEntry(
            command_id='unknown',
            mission_idx=0,
            action='unknown',
            robot_id='unknown',
            params={},
            state='PLANNING_FAILED',
            reason=data.get('reason', 'Unknown planning error'),
            created_at=time.time(),
            started_at=0.0,
            completed_at=time.time()
        )
        self._history.append(entry)
        if len(self._history) > 10:
            self._history.pop(0)
        self._publish_mission_status(entry)  # publish this standalone
        self._publish_queue_status()
```

Actually, for planning failures from `/llm/decision_status`, the entry doesn't
belong to a specific mission. Better approach: publish a standalone mission_status
with `state: "PLANNING_FAILED"` and `reason: "..."` so the CLI can show it,
without adding it to the queue. The `_publish_mission_status` function should
support publishing for non-active missions too.

### Keep existing functionality

All other methods (odom callbacks, fleet state callback, GPS-to-XY conversion,
home pose loading, waypoint resolution, etc.) remain unchanged.

---

## 5. CLI (`cli.py`)

### Changes

**Remove**:
- `self.waiting_for_response = False`
- `self.raw_decision_sub` subscription (raw LLM responses are internal detail)
- `_raw_decision_cb` method

**Add subscriptions**:
```python
self.mission_status_sub = self.create_subscription(
    String, '/llm/mission_status', self._mission_status_cb, 10)
self.mission_queue_sub = self.create_subscription(
    String, '/llm/mission_queue', self._mission_queue_cb, 10)
self.command_status_sub = self.create_subscription(
    String, '/llm/command_status', self._command_status_cb, 10)
```

**Add state**:
```python
self._command_status = {}       # command_id → status info
self._latest_queue = None       # mission_queue JSON
```

**`_command_status_cb`** (new):
```python
def _command_status_cb(self, msg):
    try:
        data = json.loads(msg.data)
    except json.JSONDecodeError:
        return
    cid = data.get('command_id', '')[:8]
    status = data.get('status', '')
    
    if status == 'queued':
        print(f"\n\033[1;33m[Queue]\033[0m Command queued ({cid})")
    elif status == 'planning':
        print(f"\n\033[1;33m[LLM]\033[0m Processing... ({cid})")
    elif status == 'done':
        print(f"\n\033[1;32m[LLM]\033[0m Response received ({cid})")
    elif status == 'failed':
        reason = data.get('reason', 'Unknown error')
        print(f"\n\033[1;31m[LLM Error]\033[0m {reason} ({cid})")
    elif status == 'skipped':
        print(f"\n\033[1;33m[Skipped]\033[0m {data.get('reason', '')}")
    
    print("\033[1;32mhusky>\033[0m ", end='', flush=True)
```

**`_mission_status_cb`** (new):
```python
def _mission_status_cb(self, msg):
    try:
        data = json.loads(msg.data)
    except json.JSONDecodeError:
        return
    
    state = data.get('state', '')
    robot = data.get('robot_id', '?')
    action = data.get('action', '?')
    elapsed = data.get('elapsed', 0)
    
    if state == 'EXECUTING':
        print(f"\n\033[1;34m[Mission]\033[0m {robot}: {action}... (running {elapsed}s)")
    elif state == 'SUCCEEDED':
        print(f"\n\033[1;32m[Mission]\033[0m {robot}: {action} → \033[1;32mSucceeded\033[0m ({elapsed}s)")
    elif state == 'EXECUTED_FAILED':
        reason = data.get('reason', 'Unknown error')
        print(f"\n\033[1;31m[Mission]\033[0m {robot}: {action} → \033[1;31mFailed\033[0m — {reason}")
    elif state == 'PLANNING_FAILED':
        reason = data.get('reason', 'Unknown error')
        print(f"\n\033[1;31m[Mission]\033[0m → \033[1;31mPlanning Failed\033[0m — {reason}")
    
    print("\033[1;32mhusky>\033[0m ", end='', flush=True)
```

**`_mission_queue_cb`** (new):
```python
def _mission_queue_cb(self, msg):
    try:
        self._latest_queue = json.loads(msg.data)
    except json.JSONDecodeError:
        pass
```

**Modify `_show_status`** — show queue info:
```python
def _show_status(self):
    # Existing fleet state display
    if not self.fleet_state:
        print("\033[1;33m[No fleet data]\033[0m")
    else:
        for i, robot_id in enumerate(self.fleet_state.robot_ids):
            state = self.fleet_state.states[i]
            status = []
            if state.emergency_stop:
                status.append('EMERGENCY')
            if state.waiting:
                status.append('WAITING')
            if state.mission_active:
                status.append('NAVIGATING')
            if state.idle:
                status.append('IDLE')
            print(f"\033[1;36m{robot_id}:\033[0m {', '.join(status) or 'UNKNOWN'}")

    # Queue display
    if self._latest_queue:
        q = self._latest_queue
        active = q.get('active')
        pending = q.get('pending', [])
        history = q.get('history', [])

        if active:
            print(f"  \033[1;33mActive:\033[0m {active['robot_id']}: {active['action']} "
                  f"({active['state']}, {active.get('elapsed', 0)}s)")
        if pending:
            print(f"  \033[1;33mPending:\033[0m {len(pending)} mission(s)")
            for p in pending:
                print(f"    {p['robot_id']}: {p['action']} ({p['state']})")
        if history:
            print(f"  \033[1;33mRecent:\033[0m")
            for h in history[-3:]:
                color = '\033[1;32m' if h['state'] == 'SUCCEEDED' else '\033[1;31m'
                print(f"    {color}{h['action']}\033[0m on {h['robot_id']} → "
                      f"{h['state']} ({h.get('duration', 0)}s)")
                if h.get('reason'):
                    print(f"      {h['reason']}")
```

**Modify `_send_command`** — remove `waiting_for_response`, add command_id tracking:
```python
def _send_command(self, cmd):
    msg = String()
    msg.data = cmd
    self.command_pub.publish(msg)
    print("\033[1;34m[Sent]\033[0m", cmd)
```

---

## Build & Verify

```bash
cd /root/ros_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --packages-select husky_llm_bridge
```

Then launch and test:
```bash
source install/setup.bash
# Start the pipeline, send 3 commands in quick succession from CLI
# 1. "go to x=5, y=0"
# 2. "go to docking station"
# 3. "move forward 2 meters"
# Expected: 1 executes, 2 shows PLANNING_FAILED, 3 executes immediately after
```
