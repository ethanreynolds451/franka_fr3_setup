'''
Simple script to control Franka joints using individual keyboard inputs
Since this is a standalone script, make sure to source both ROS2 and the workspace wherever it is run

This script provides control of a simulated Franka robot through the joint trajectory action server. 
Modifications may be needed for a real robot but the general structure would probably be similar

The script will read keyboard inputs to determine joint movements then send them to the robot

Same effect as using commands like the following: 
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: [fr3_joint1, fr3_joint2, fr3_joint3, fr3_joint4, fr3_joint5, fr3_joint6, fr3_joint7], points: [{positions: [0.0, -0.5, 0.0, -2.0, 0.0, 1.5, 0.7], time_from_start: {sec: 1, nanosec: 0}}]}}"

'''

# Import required ROS modules and message types
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# Import additional modules for keyboard input handling
# May need to pip install pyinput if not already available
from pynput import keyboard

class FrankaKeyboardTrajectoryController(Node):

    def __init__(self):
        super().__init__('franka_keyboard_trajectory_controller')

        # List of joint commands corresponding to listed inputs
        self.joint_commands = ['rest', 'increase', 'decrease']  # Example command types for each joint

        # Maps joint names to their corresponding keyboard inputs
        self.joint_key_map = {
            'fr3_joint1'  :   ['1', 'q', 'a'],
            'fr3_joint2'  :   ['2', 'w', 's'],
            'fr3_joint3'  :   ['3', 'e', 'd'],
            'fr3_joint4'  :   ['4', 'r', 'f'],
            'fr3_joint5'  :   ['5', 't', 'g'],
            'fr3_joint6'  :   ['6', 'y', 'h'],
            'fr3_joint7'  :   ['7', 'u', 'j']
        }

        # Declare arm control parameters
        self.arm_increment = 0.05                                            # Define the increment for joint angle changes
        self.arm_rest_position = [0.0, -0.8, 0.0, -2.4, 0.0, 1.6, -0.8]        # Define rest position for the robot
        self.arm_current_position = self.arm_rest_position.copy()               # Initialize current position to rest
        self.arm_rest_duration = 1.0                                        # How long to give the arm to return to rest
        self.arm_increment_duration = 0.1                                   # How long to give the arm to execute a joint increment

        # Connect to the active action client for joint trajectory
        # Need to make sure the controller is activated and associated the robot / simulation being controlled
        self._arm_action_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/joint_trajectory_controller/follow_joint_trajectory'
        )
        

        # Before starting to listen for keyboard input, wait for the action server to be available
        while not self._arm_action_client.wait_for_server(timeout_sec=1.0):
            # Hold the program here until there is an action server available
            self.get_logger().info('Waiting for arm action server to be available...')

        self.get_logger().info('Action server is now available. Starting keyboard listener.')


        # Start a pyinput thread with callback to listen for keyboard input
        listener = keyboard.Listener(on_press=self.on_key_press)
        listener.start()

        self.get_logger().info('Keyboard listener started. Use the following keys to control the robot:')
        for joint, keys in self.joint_key_map.items():
            self.get_logger().info(f'{joint}: {keys[0]} (rest), {keys[1]} (increase), {keys[2]} (decrease)')


    # Callback when a key is pressed
    # It can only handle one at a time using this approach
    def on_key_press(self, key):
        key_char = key.char if hasattr(key, 'char') else None
        if not key_char:
            return  # Ignore non-character keys
        for joint, keys in self.joint_key_map.items():
            if key_char in keys: 
                command = self.joint_commands[keys.index(key_char)]  # Get the command type based on the key pressed
                self.update_arm_trajectory(joint, command)  # Update the trajectory based on the command
                if command == 'rest':
                    self.send_arm_trajectory_goal(self.arm_current_position, duration_seconds=self.arm_rest_duration)
                else: 
                    self.send_arm_trajectory_goal(self.arm_current_position, duration_seconds=self.arm_increment_duration) 
                return  # Only one key will be processed at once so don't have to keep looing
        return  # Ignore keys that are not mapped to any joint command


    # Should probably merge the functions since this is kindof messy but this will work for now
    def update_arm_trajectory(self, joint_name, command):
        index = list(self.joint_key_map.keys()).index(joint_name)  # Get the index of the joint to update
        if command == 'rest':
            self.arm_current_position[index] = self.arm_rest_position[index]  # Reset to rest position
        elif command == 'increase':
            if self.arm_current_position[index] + self.arm_increment <= 3.14:  # Check upper limit
                self.arm_current_position[index] += self.arm_increment  # Increase joint angle
        elif command == 'decrease':
            if self.arm_current_position[index] - self.arm_increment >= -3.14:  # Check lower limit
                self.arm_current_position[index] -= self.arm_increment  # Decrease joint angle
  
    def send_arm_trajectory_goal(self, target_positions, duration_seconds):
        # First check if the action server is available before sending a goal
        if not self._arm_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Action server not available.')
            while not self._arm_action_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().info('Waiting for action server to be available...')
            self.get_logger().info('Action server is now available. Sending goal.')

        # Create a goal object
        goal_msg = FollowJointTrajectory.Goal()
        
        # Add joint names, must match the Franka yaml config
        joints = list(self.joint_key_map.keys())
        goal_msg.trajectory.joint_names = joints

        # Synchronize clock stamping for immediate simulation time 
        goal_msg.trajectory.header.stamp.sec = 0
        goal_msg.trajectory.header.stamp.nanosec = 0

        # Create waypoint target object and assign target positions
        point = JointTrajectoryPoint()
        point.positions = target_positions
        
        # Give the arm the right amount of time to travel smoothly
        # Uses a duration ROS2 obkect
        point.time_from_start = Duration(sec=int(duration_seconds), nanosec=0)
        
        # Add the point into the trajectory goal message
        goal_msg.trajectory.points.append(point)
        self.get_logger().info('Sending trajectory goal to Franka robot...')
        
        # Send goal and create a future callback to handle the response
        self._send_goal_future = self._arm_action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    # Shared callbacks for both action clients since they have the same message types
    # Added error handling

    # Callback to handle periodic feedback from the controller
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # self.get_logger().info(f'Current trajectory execution time: {feedback.actual.time_from_start.sec} seconds')
        pass

    # Callback when goal feedback is recieved from the controller 
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by controller.')
            return

        self.get_logger().info('Goal accepted. Executing trajectory...')
        # Request the result for the goal regardless of gripper mode
        try:
            self._get_result_future = goal_handle.get_result_async()
            self._get_result_future.add_done_callback(self.get_result_callback)
        except Exception:
            # Some action servers may not provide a result; ignore in that case
            pass

    # Callback when the trajectory execution is completed or fails with result
    def get_result_callback(self, future):
        res = future.result()
        result_obj = getattr(res, 'result', None)

        if result_obj is None:
            self.get_logger().warning('No result payload received from action server.')
            return

        # FollowJointTrajectory results provide `error_code`.
        if hasattr(result_obj, 'error_code'):
            if result_obj.error_code != 0:
                self.get_logger().error(f'Trajectory execution failed. error_code={result_obj.error_code}')
            else:
                self.get_logger().info('Trajectory execution finished successfully!')
            return

        # Fallback: log the result object for debugging
        self.get_logger().info(f'Action result received: {result_obj}')



def main(args=None):
    rclpy.init(args=args)
    
    # Start the keyboard control node
    trajectory_client = FrankaKeyboardTrajectoryController()

    # Spin the node to listen for inputs and process callbacks
    rclpy.spin(trajectory_client)

    # Clean exit when node shut down in terminal
    trajectory_client.destroy_node()
    rclpy.shutdown()


# Allow script to be executed directly
if __name__ == '__main__':
    main()
