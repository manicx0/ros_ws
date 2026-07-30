#!/usr/bin/env python3
import json
import readline
import sys
import threading
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from husky_msgs.msg import FleetState, GoalEvent


class HuskyCLI(Node):
    def __init__(self):
        super().__init__('husky_cli')
        self.command_pub = self.create_publisher(String, '/llm/command', 10)
        self.fleet_state_sub = self.create_subscription(
            FleetState, '/fleet/robot_states', self._fleet_state_cb, 10)
        self.decision_status_sub = self.create_subscription(
            String, '/llm/decision_status', self._decision_status_cb, 10)
        self.goal_event_sub = self.create_subscription(
            GoalEvent, '/fleet/goal_events', self._goal_event_cb, 10)
        self.command_status_sub = self.create_subscription(
            String, '/llm/command_status', self._command_status_cb, 10)
        self.mission_status_sub = self.create_subscription(
            String, '/llm/mission_status', self._mission_status_cb, 10)
        self.mission_queue_sub = self.create_subscription(
            String, '/llm/mission_queue', self._mission_queue_cb, 10)
        self.natural_language_sub = self.create_subscription(
            String, '/llm/natural_language_response', self._natural_language_cb, 10)
        self.fleet_state = None
        self._latest_queue = None
        self._last_event_key = None
        self._last_event_time = 0.0

    def _fleet_state_cb(self, msg):
        self.fleet_state = msg

    def _writeln(self, text):
        sys.stdout.write(f"\r{text}\n\033[1;32mhusky>\033[0m ")
        sys.stdout.flush()

    def _decision_status_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            data = msg.data
        self._writeln(f"\033[1;31m[LLM Error]\033[0m {data}")

    def _goal_event_cb(self, msg):
        now = time.time()
        key = (msg.robot_id, msg.type)
        elapsed = now - self._last_event_time
        if key != self._last_event_key or elapsed > 2.0:
            self._writeln(f"\033[1;34m[Event]\033[0m {msg.robot_id}: {msg.type}")
        self._last_event_key = key
        self._last_event_time = now

    def _command_status_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        cid = data.get('command_id', '')[:8]
        status = data.get('status', '')

        if status == 'queued':
            self._writeln(f"\033[1;33m[Queue]\033[0m Command queued ({cid})")
        elif status == 'planning':
            self._writeln(f"\033[1;33m[LLM]\033[0m Processing... ({cid})")
        elif status == 'done':
            self._writeln(f"\033[1;32m[LLM]\033[0m Response received ({cid})")
        elif status == 'failed':
            reason = data.get('reason', 'Unknown error')
            self._writeln(f"\033[1;31m[LLM Error]\033[0m {reason} ({cid})")
        elif status == 'skipped':
            self._writeln(f"\033[1;33m[Skipped]\033[0m {data.get('reason', '')}")

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
            self._writeln(f"\033[1;34m[Mission]\033[0m {robot}: {action}... (running {elapsed}s)")
        elif state == 'SUCCEEDED':
            self._writeln(f"\033[1;32m[Mission]\033[0m {robot}: {action} → \033[1;32mSucceeded\033[0m ({elapsed}s)")
        elif state == 'EXECUTED_FAILED':
            self._writeln(f"\033[1;33m[Mission]\033[0m {robot}: {action} → Completed")
        elif state == 'PLANNING_FAILED':
            self._writeln(f"\033[1;31m[Mission]\033[0m → \033[1;31mPlanning Failed\033[0m — {data.get('reason', '')}")

    def _mission_queue_cb(self, msg):
        try:
            self._latest_queue = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _natural_language_cb(self, msg):
        self._writeln(f"\033[1;35m[LLM]\033[0m {msg.data}")

    def run(self):
        print("\033[1;36mHusky Fleet CLI\033[0m (type 'help' for commands)\n")
        while True:
            try:
                cmd = input("\033[1;32mhusky>\033[0m ").strip()
                if cmd in ('quit', 'exit'):
                    break
                elif cmd == 'status':
                    self._show_status()
                elif cmd == 'help':
                    self._show_help()
                elif cmd == 'clear':
                    print("\033[2J\033[H")
                elif cmd:
                    self._send_command(cmd)
            except (EOFError, KeyboardInterrupt):
                break
        print("\nGoodbye!")

    def _send_command(self, cmd):
        msg = String()
        msg.data = cmd
        self.command_pub.publish(msg)
        self._writeln(f"\033[1;34m[Sent]\033[0m {cmd}")

    def _show_status(self):
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

    def _show_help(self):
        print("""
\033[1;33mCommands:\033[0m
  <text>    Send command to LLM (e.g., "send robot to x=3,y=0")
  status    Show current fleet state + mission queue
  help      Show this help
  clear     Clear screen
  quit      Exit CLI
        """)


def main():
    rclpy.init()
    cli = HuskyCLI()
    spinner = threading.Thread(target=rclpy.spin, args=(cli,), daemon=True)
    spinner.start()
    try:
        cli.run()
    finally:
        cli.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
