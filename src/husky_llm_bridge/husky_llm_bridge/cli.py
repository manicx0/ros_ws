#!/usr/bin/env python3
import readline
import threading
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
        self.raw_decision_sub = self.create_subscription(
            String, '/llm/raw_decision', self._raw_decision_cb, 10)
        self.fleet_state = None
        self.waiting_for_response = False

    def _fleet_state_cb(self, msg):
        self.fleet_state = msg

    def _decision_status_cb(self, msg):
        print(f"\n\033[1;31m[LLM Error]\033[0m {msg.data}")
        print("\033[1;32mhusky>\033[0m ", end='', flush=True)

    def _goal_event_cb(self, msg):
        print(f"\n\033[1;34m[Event]\033[0m {msg.robot_id}: {msg.type}")
        print("\033[1;32mhusky>\033[0m ", end='', flush=True)

    def _raw_decision_cb(self, msg):
        if self.waiting_for_response:
            self.waiting_for_response = False
            response = msg.data.strip()
            if response.startswith('{'):
                print(f"\n\033[1;35m[LLM]\033[0m Processing mission...")
            else:
                print(f"\n\033[1;35m[LLM]\033[0m {response}")
            print("\033[1;32mhusky>\033[0m ", end='', flush=True)

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
        self.waiting_for_response = True
        print("\033[1;34m[Sent]\033[0m", cmd)

    def _show_status(self):
        if not self.fleet_state:
            print("\033[1;33m[No fleet data]\033[0m")
            return
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

    def _show_help(self):
        print("""
\033[1;33mCommands:\033[0m
  <text>    Send command to LLM (e.g., "send robot to x=3,y=0")
  status    Show current fleet state
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
