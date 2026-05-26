'''
Keyboard control for a real Franka FR3 arm via the joint trajectory action server.

Key changes vs. the simulation version:
  1. Goals are queued and only sent once the previous one finishes, preventing
     the controller from receiving overlapping goals (a common source of
     discontinuity / goal-rejection errors on real hardware).
  2. The trajectory header stamp is set to the current ROS clock time so the
     controller can schedule execution correctly.
  3. Every JointTrajectoryPoint now includes zero velocities and accelerations,
     which many real controllers require to avoid "path tolerance violated" or
     discontinuity errors at the endpoint.
  4. The increment duration is stored as separate sec/nanosec fields so the
     fractional part (e.g. 0.5 s → 500 000 000 ns) is never silently truncated.
  5. A threading lock guards self.arm_current_position so keypress callbacks
     (which run on a background thread) cannot race with the ROS spin thread.

Source / run instructions are the same as before — make sure to source both
ROS 2 and your workspace, then:
    python3 franka_keyboard_controller.py
'''

import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from pynput import keyboard  # pip install pynput if missing


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def seconds_to_duration(secs: float) -> Duration:
    """Convert a float number of seconds to a ROS Duration message."""
    whole = int(secs)
    nano  = int(round((secs - whole) * 1_000_000_000))
    return Duration(sec=whole, nanosec=nano)


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class FrankaKeyboardTrajectoryController(Node):

    # Joint names must match the controller YAML on the real robot
    JOINT_NAMES = [
        'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
        'fr3_joint5', 'fr3_joint6', 'fr3_joint7',
    ]

    # key → (joint_index, command)   command: 'rest' | 'increase' | 'decrease'
    KEY_MAP = {
        '1': (0, 'rest'),    'q': (0, 'increase'),  'a': (0, 'decrease'),
        '2': (1, 'rest'),    'w': (1, 'increase'),  's': (1, 'decrease'),
        '3': (2, 'rest'),    'e': (2, 'increase'),  'd': (2, 'decrease'),
        '4': (3, 'rest'),    'r': (3, 'increase'),  'f': (3, 'decrease'),
        '5': (4, 'rest'),    't': (4, 'increase'),  'g': (4, 'decrease'),
        '6': (5, 'rest'),    'y': (5, 'increase'),  'h': (5, 'decrease'),
        '7': (6, 'rest'),    'u': (6, 'increase'),  'j': (6, 'decrease'),
    }

    REST_POSITION       = [0.0, -0.8, 0.0, -2.4, 0.0, 1.6, -0.8]
    ARM_INCREMENT       = 0.05   # radians per keypress
    JOINT_LIMIT         = 3.14   # soft ±limit used for clamping
    REST_DURATION       = 1.0    # seconds for a rest move
    INCREMENT_DURATION  = 0.5    # seconds for an increment move
                                 # (≥0.1 s recommended on real hardware;
                                 #  tune to taste — shorter = more responsive
                                 #  but may exceed velocity limits)

    def __init__(self):
        super().__init__('franka_keyboard_trajectory_controller')

        self._position_lock   = threading.Lock()
        self._goal_in_flight  = False          # True while a goal is executing
        self._pending_goal    = None           # (positions, duration) or None

        self._current_position = list(self.REST_POSITION)

        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory',
        )

        self.get_logger().info('Waiting for action server…')
        self._action_client.wait_for_server()
        self.get_logger().info('Action server ready.')

        # Print key bindings
        self.get_logger().info('Keyboard bindings:')
        for joint_idx, name in enumerate(self.JOINT_NAMES):
            keys = [k for k, (ji, _) in self.KEY_MAP.items() if ji == joint_idx]
            rest_k = [k for k, (ji, cmd) in self.KEY_MAP.items() if ji == joint_idx and cmd == 'rest']
            inc_k  = [k for k, (ji, cmd) in self.KEY_MAP.items() if ji == joint_idx and cmd == 'increase']
            dec_k  = [k for k, (ji, cmd) in self.KEY_MAP.items() if ji == joint_idx and cmd == 'decrease']
            self.get_logger().info(
                f'  {name}: {rest_k[0]} (rest)  {inc_k[0]} (increase)  {dec_k[0]} (decrease)'
            )

        listener = keyboard.Listener(on_press=self._on_key_press)
        listener.start()
        self.get_logger().info('Listening for key presses.')

    # ------------------------------------------------------------------
    # Keyboard callback  (runs on pynput thread)
    # ------------------------------------------------------------------

    def _on_key_press(self, key):
        char = getattr(key, 'char', None)
        if char is None or char not in self.KEY_MAP:
            return

        joint_idx, command = self.KEY_MAP[char]

        with self._position_lock:
            if command == 'rest':
                self._current_position[joint_idx] = self.REST_POSITION[joint_idx]
                duration = self.REST_DURATION
            elif command == 'increase':
                new_val = self._current_position[joint_idx] + self.ARM_INCREMENT
                self._current_position[joint_idx] = min(new_val, self.JOINT_LIMIT)
                duration = self.INCREMENT_DURATION
            else:  # decrease
                new_val = self._current_position[joint_idx] - self.ARM_INCREMENT
                self._current_position[joint_idx] = max(new_val, -self.JOINT_LIMIT)
                duration = self.INCREMENT_DURATION

            positions = list(self._current_position)

        self._queue_goal(positions, duration)

    # ------------------------------------------------------------------
    # Goal queuing  (called from pynput thread, safe via GIL + flag)
    # ------------------------------------------------------------------

    def _queue_goal(self, positions, duration):
        """
        If no goal is currently executing, send immediately.
        Otherwise, store as pending — it will be sent as soon as the
        current goal finishes.  Only the *latest* pending goal is kept,
        so rapid keypresses do not build up an unbounded queue.
        """
        if not self._goal_in_flight:
            self._goal_in_flight = True
            self._send_goal(positions, duration)
        else:
            self._pending_goal = (positions, duration)

    # ------------------------------------------------------------------
    # Goal construction & dispatch
    # ------------------------------------------------------------------

    def _send_goal(self, positions, duration):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self.JOINT_NAMES)

        # Stamp with current ROS time so the real controller can schedule
        # execution correctly (sim can use zeros; hardware cannot).
        now = self.get_clock().now().to_msg()
        goal.trajectory.header.stamp = now

        n = len(self.JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions      = list(positions)
        point.velocities     = [0.0] * n   # required by many real controllers
        point.accelerations  = [0.0] * n   # ditto
        point.time_from_start = seconds_to_duration(duration)

        goal.trajectory.points.append(point)

        self.get_logger().info(
            f'Sending goal: {[round(p, 3) for p in positions]}  '
            f'duration={duration:.2f}s'
        )

        future = self._action_client.send_goal_async(
            goal,
            feedback_callback=self._feedback_callback,
        )
        future.add_done_callback(self._goal_response_callback)

    # ------------------------------------------------------------------
    # Action callbacks
    # ------------------------------------------------------------------

    def _feedback_callback(self, feedback_msg):
        # Uncomment to log live joint positions during execution:
        # fb = feedback_msg.feedback
        # self.get_logger().info(f'actual: {fb.actual.positions}')
        pass

    def _goal_response_callback(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Goal rejected by controller.')
            self._finish_goal()
            return
        self.get_logger().info('Goal accepted, executing…')
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        result = future.result().result
        if result.error_code == 0:
            self.get_logger().info('Trajectory complete.')
        else:
            self.get_logger().error(
                f'Trajectory failed. error_code={result.error_code}  '
                f'error_string={result.error_string}'
            )
        self._finish_goal()

    def _finish_goal(self):
        """Called when the current goal finishes.  Send any pending goal."""
        pending = self._pending_goal
        self._pending_goal = None
        if pending is not None:
            positions, duration = pending
            self._send_goal(positions, duration)
        else:
            self._goal_in_flight = False


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = FrankaKeyboardTrajectoryController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
