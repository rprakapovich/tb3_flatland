#!/usr/bin/env python3
"""
umbreon_controller.py — ROS2 node that runs the Umbreon roborace algorithm
in Flatland simulation.

Sensor topics (sensor_msgs/LaserScan, 1 ray each):
    /umbreon/s0  L-Out  45° left
    /umbreon/s1  L-Fwd  0° straight, left of centre
    /umbreon/s2  R-Fwd  0° straight, right of centre
    /umbreon/s3  R-Out  45° right

Control output:
    /umbreon/cmd_vel  (geometry_msgs/Twist)
        linear.x  = target speed (m/s)
        angular.z = -tan(steer_angle) / WHEELBASE * linear_x
                    (bicycle model, positive cmd_vel.z = left turn)
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# ── Algorithm constants (mirrors Umbreon_roborace.ino) ───────────────────────
FRONT_OBSTACLE_DIST = 600    # cm×10 = 60 cm
SIDE_OPEN_DIST      = 1000   # cm×10 = 100 cm
ALL_CLOSE_DIST      = 800    # cm×10 = 80 cm
CLOSE_FRONT_DIST    = 201    # cm×10 = ~20 cm
LOOP_MS             = 40     # ms — control period

# ── Car physics ───────────────────────────────────────────────────────────────
WHEELBASE     = 0.18              # m
MAX_STEER_RAD = math.radians(28)  # physical max steering angle

MAX_RANGE_CM10 = 9999   # reported when sensor sees nothing


def _range_to_cm10(r: float) -> int:
    """Convert LaserScan range (metres, possibly inf/nan) to cm×10."""
    if not math.isfinite(r) or r <= 0.0:
        return MAX_RANGE_CM10
    return min(int(r * 1000.0), MAX_RANGE_CM10)


def work(state: dict, s: list):
    """
    Direct port of work() from Umbreon_roborace.ino / sim.py.
    state : {'stuck_time': int, 'turns': float, 'v': float}
    s     : [s0, s1, s2, s3]  in cm×10
    Returns (steer_cmd, target_speed_ms)
    """
    f_l = s[1] < FRONT_OBSTACLE_DIST
    f_r = s[2] < FRONT_OBSTACLE_DIST

    # Steering diff
    if s[0] > SIDE_OPEN_DIST and s[3] > SIDE_OPEN_DIST:
        diff = 800
    else:
        diff = s[3] - s[0]

    if all(x < ALL_CLOSE_DIST for x in s):
        diff = 800

    # Diagonal correction
    diff += (s[2] - s[1]) // 3

    # Speed
    how_clear = int(f_l) + int(f_r)
    if how_clear == 0:
        coef, spd = 0.3, 2.7
    else:
        coef, spd = 0.7, 0.8

    steer = int(max(-1000, min(1000, int(diff * coef))))

    # Stuck detection
    c_fl = s[1] < CLOSE_FRONT_DIST
    c_fr = s[2] < CLOSE_FRONT_DIST
    if c_fl or c_fr or state['v'] < 0.1:
        state['stuck_time'] += 1
    else:
        state['stuck_time'] = 0

    if state['stuck_time'] > 25:
        steer = 0
        spd   = -0.5
        state['stuck_time'] = 0

    # U-turn detection
    state['turns'] += diff * state['v'] / -1000.0
    state['turns']  = float(max(-1500.0, min(50.0, state['turns'])))

    if state['turns'] < -18.0:
        steer          = 1000
        spd            = -0.5
        state['turns'] = 0.0

    return steer, spd


class UmbreonController(Node):
    def __init__(self):
        super().__init__('umbreon_controller')

        self._s = [MAX_RANGE_CM10] * 4   # latest sensor readings (cm×10)
        self._state = {'stuck_time': 0, 'turns': 0.0, 'v': 0.0}
        self._current_v = 0.0

        ns = 'umbreon'

        for i in range(4):
            self.create_subscription(
                LaserScan,
                f'/{ns}/s{i}',
                self._make_cb(i),
                10)

        self._pub = self.create_publisher(Twist, f'/{ns}/cmd_vel', 10)

        period = LOOP_MS / 1000.0
        self.create_timer(period, self._control_tick)
        self.get_logger().info('Umbreon controller started.')

    def _make_cb(self, idx: int):
        def cb(msg: LaserScan):
            r = msg.ranges[0] if msg.ranges else float('inf')
            self._s[idx] = _range_to_cm10(r)
        return cb

    def _control_tick(self):
        self._state['v'] = self._current_v
        steer_cmd, target_spd = work(self._state, self._s)

        # Bicycle model → differential drive cmd_vel
        steer_angle = (steer_cmd / 1000.0) * MAX_STEER_RAD
        # steer_cmd > 0 → right → negative angular.z in ROS
        angular_z = -(math.tan(steer_angle) / WHEELBASE) * target_spd

        twist = Twist()
        twist.linear.x  = float(target_spd)
        twist.angular.z = float(angular_z)
        self._pub.publish(twist)

        # Rough speed estimate from last command (no tachometer in sim)
        self._current_v = target_spd


def main(args=None):
    rclpy.init(args=args)
    node = UmbreonController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
