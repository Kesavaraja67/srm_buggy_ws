#!/usr/bin/env python3
"""
arrow_teleop.py — SRM Autonomous Buggy
──────────────────────────────────────────────────────────────
Custom keyboard teleop using ARROW KEYS.
Designed for realistic car-like control:
  ↑  = Forward (hold to accelerate)
  ↓  = Backward / Brake
  ←  = Gentle turn left  (small angular step per press)
  →  = Gentle turn right (small angular step per press)
  SPACE = Emergency STOP
  q = Quit

Physics model:
  - Linear speed builds gradually (acceleration)
  - Each LEFT/RIGHT press adds a small steering increment
  - Steering returns toward centre when not pressed
  - Releasing forward key: speed decays (friction)
"""

import sys
import curses
import threading
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# ── Tunable constants ────────────────────────────────────────
MAX_LINEAR   =  2.0   # m/s maximum forward speed
MAX_BACKWARD = -1.0   # m/s maximum backward speed
MAX_ANGULAR  =  0.8   # rad/s maximum turn rate
ACCEL        =  0.15  # m/s² — speed increase per key press
DECEL        =  0.10  # m/s  — natural deceleration per tick
STEER_STEP   =  0.15  # rad/s added per LEFT/RIGHT press
STEER_RETURN =  0.08  # rad/s steering centre-return per tick (self-centering)
PUBLISH_HZ   =  50    # control loop rate (50Hz = 20ms period for low lag)
# ────────────────────────────────────────────────────────────


class ArrowTeleopNode(Node):

    def __init__(self):
        super().__init__('arrow_teleop_node')   # type: ignore[call-arg]
        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._linear  = 0.0
        self._angular = 0.0
        self._lock    = threading.Lock()

    def apply(self, linear: float, angular: float):
        with self._lock:
            self._linear  = linear
            self._angular = angular

    def publish_current(self):
        msg = Twist()
        with self._lock:
            msg.linear.x  = self._linear
            msg.angular.z = self._angular
        self._pub.publish(msg)

    def stop(self):
        msg = Twist()
        self._pub.publish(msg)


def run_curses(stdscr, node: ArrowTeleopNode):
    """Main curses loop — reads arrow keys and updates the buggy."""
    curses.curs_set(0)
    stdscr.nodelay(True)       # non-blocking key read
    stdscr.timeout(20)         # 20ms = 50Hz poll match

    curses.start_color()
    curses.init_pair(1, curses.COLOR_GREEN,  curses.COLOR_BLACK)
    curses.init_pair(2, curses.COLOR_YELLOW, curses.COLOR_BLACK)
    curses.init_pair(3, curses.COLOR_RED,    curses.COLOR_BLACK)
    curses.init_pair(4, curses.COLOR_CYAN,   curses.COLOR_BLACK)

    linear  = 0.0
    angular = 0.0

    def draw(status='READY'):
        stdscr.clear()
        h, w = stdscr.getmaxyx()
        title = '  SRM Autonomous Buggy — Arrow Key Control  '
        stdscr.addstr(0, max(0, (w - len(title))//2), title, curses.A_BOLD | curses.color_pair(4))
        stdscr.addstr(2, 2, '┌─────────────────────────────┐')
        stdscr.addstr(3, 2, '│   ↑  Forward                │')
        stdscr.addstr(4, 2, '│   ↓  Backward / Brake       │')
        stdscr.addstr(5, 2, '│   ←  Gentle Left Turn       │')
        stdscr.addstr(6, 2, '│   →  Gentle Right Turn      │')
        stdscr.addstr(7, 2, '│  SPC  Emergency STOP        │')
        stdscr.addstr(8, 2, '│   q   Quit                  │')
        stdscr.addstr(9, 2, '└─────────────────────────────┘')

        spd_bar = '█' * int(abs(linear) / MAX_LINEAR * 15)
        ang_bar = '█' * int(abs(angular) / MAX_ANGULAR * 10)

        col_spd = curses.color_pair(1) if linear >= 0 else curses.color_pair(3)
        stdscr.addstr(11, 2, f'Speed : {linear:+.2f} m/s  [{spd_bar:<15}]', col_spd)
        col_ang = curses.color_pair(2) if angular >= 0 else curses.color_pair(3)
        stdscr.addstr(12, 2, f'Steer : {angular:+.2f} rad/s [{ang_bar:<10}]', col_ang)

        col_st = curses.color_pair(1) if status == 'MOVING' else \
                 curses.color_pair(3) if status == 'STOPPED' else curses.color_pair(2)
        try:
            stdscr.addstr(14, 2, f'Status: {status}', col_st | curses.A_BOLD)
        except curses.error:
            pass
        stdscr.refresh()

    draw()

    linear: float       = 0.0   # forward/backward speed  (m/s)
    steer_target: float = 0.0   # desired steering angle   (rad/s) — accumulates on L/R

    # Terminals can only report ONE key at a time. When holding UP + LEFT,
    # the terminal alternates or only sends the last-pressed key.
    # We use a "hold" timer: ANY key press keeps both axes alive.
    last_any_key_time: float   = time.time()
    last_linear_input: float   = time.time()
    last_steer_input: float    = time.time()
    HOLD_TIME: float           = 0.5   # 500ms — hold speed while user is active

    while rclpy.ok():
        now = time.time()

        # 1. Read ALL available keys in the buffer this iteration
        keys_pressed: set = set()
        while True:
            key = stdscr.getch()
            if key == -1:
                break
            keys_pressed.add(key)

        any_key = len(keys_pressed) > 0

        if ord('q') in keys_pressed or ord('Q') in keys_pressed:
            node.stop()
            break

        if ord(' ') in keys_pressed:
            linear = 0.0
            steer_target = 0.0
            node.stop()
            draw('STOPPED')
            continue

        # Track when ANY key was last pressed
        if any_key:
            last_any_key_time = now

        # 2. Extract specific actions from the key set
        up    = curses.KEY_UP    in keys_pressed
        down  = curses.KEY_DOWN  in keys_pressed
        left  = curses.KEY_LEFT  in keys_pressed
        right = curses.KEY_RIGHT in keys_pressed

        status = 'IDLE'

        # 3. Handle Throttle / Brake
        if up:
            linear = min(linear + ACCEL, MAX_LINEAR)
            last_linear_input = now
            status = 'MOVING'
        elif down:
            if linear > 0.05:
                linear = max(linear - ACCEL * 2, 0.0)   # brake
                status = 'BRAKING'
            else:
                linear = max(linear - ACCEL, MAX_BACKWARD)   # reverse
                status = 'REVERSE'
            last_linear_input = now

        # 4. Handle Steering
        if left:
            steer_target = min(steer_target + STEER_STEP, MAX_ANGULAR)
            last_steer_input = now
        elif right:
            steer_target = max(steer_target - STEER_STEP, -MAX_ANGULAR)
            last_steer_input = now

        # 5. Decay logic — only decay an axis if:
        #    a) That axis key is not pressed, AND
        #    b) No keys at all have been pressed for HOLD_TIME
        #       (this keeps speed alive while the other axis is being used)
        idle_time = now - last_any_key_time

        # Linear decay: only if user hasn't pressed ANY key for HOLD_TIME
        if not (up or down) and idle_time > HOLD_TIME:
            if abs(linear) > 0.02:
                linear = linear - DECEL if linear > 0 else linear + DECEL
            else:
                linear = 0.0

        # Steering self-centering: only if user hasn't pressed ANY key for HOLD_TIME
        if not (left or right) and idle_time > HOLD_TIME:
            if abs(steer_target) > STEER_RETURN:
                steer_target = steer_target - STEER_RETURN if steer_target > 0 else steer_target + STEER_RETURN
            else:
                steer_target = 0.0

        # 6. Safety and Output
        if abs(linear) < 0.01:
            linear = 0.0
        if abs(steer_target) < 0.01:
            steer_target = 0.0

        # Car rule: angular only if moving
        angular = steer_target if abs(linear) >= 0.02 else 0.0

        if abs(linear) > 0.001 or abs(angular) > 0.001:
            if status == 'IDLE':
                status = 'MOVING'

        node.apply(linear, angular)
        node.publish_current()
        draw(status)

    draw('STOPPED')
    time.sleep(0.5)


def main():
    rclpy.init()
    node = ArrowTeleopNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        curses.wrapper(run_curses, node)
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2)


if __name__ == '__main__':
    main()
