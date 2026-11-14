"""Shared GripperCommand action handling."""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from control_msgs.action import GripperCommand



class GripperNode(Node):
    """Provide the same Robotiq action interface without fixed arm targets."""

    def __init__(self):
        super().__init__('robot_pick_lift')
        inputs = {}
        for name in ('robot', 'description_package', 'arm_config', 'gripper_config', 'environment_config'):
            inputs[name] = self.declare_parameter(name, '').value
        self.profile_defaults = {}
        if any(inputs.values()):
            from .configuration import load_configuration
            _, _, _, self.profile_defaults = load_configuration(**inputs)
        for name, value in {
            'gripper_action': '/robotiq_gripper_controller/gripper_cmd',
            'gripper_open': 0.0, 'gripper_close': 0.47,
            'gripper_timeout': 10.0, 'gripper_max_effort': 50.0,
            'gripper_joint': 'robotiq_85_left_knuckle_joint',
        }.items():
            self.declare_parameter(name, self.profile_defaults.get(name, value))
        self.gripper_client = ActionClient(
            self, GripperCommand, self.get_parameter('gripper_action').value)

    def move_gripper(
        self,
        position,
        closing=False,
        timeout=None
    ):

        if timeout is None:
            timeout = self.get_parameter('gripper_timeout').value
        operation = 'CLOSE' if closing else 'OPEN'

        self.get_logger().info(
            f'Gripper {operation}: {position}'
        )

        if not self.gripper_client.wait_for_server(
            timeout_sec=5.0
        ):
            self.get_logger().error(
                'Gripper action server unavailable'
            )
            return False

        goal = GripperCommand.Goal()

        goal.command.position = position
        goal.command.max_effort = self.get_parameter('gripper_max_effort').value

        send_future = self.gripper_client.send_goal_async(
            goal
        )

        rclpy.spin_until_future_complete(
            self,
            send_future,
            timeout_sec=5.0
        )

        if not send_future.done():
            self.get_logger().error(
                'Timeout sending gripper goal'
            )
            return False

        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error(
                'Gripper goal rejected'
            )
            return False

        result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(
            self,
            result_future,
            timeout_sec=timeout
        )

        if not result_future.done():

            self.get_logger().error(
                'Gripper timeout - cancelling'
            )

            cancel_future = goal_handle.cancel_goal_async()

            rclpy.spin_until_future_complete(
                self,
                cancel_future,
                timeout_sec=2.0
            )

            return False

        wrapped_result = result_future.result()

        result = wrapped_result.result

        self.get_logger().info(
            'Gripper result: '
            f'position={result.position:.4f}, '
            f'effort={result.effort:.2f}, '
            f'stalled={result.stalled}, '
            f'reached_goal={result.reached_goal}'
        )

        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:

            self.get_logger().error(
                'Gripper action aborted'
            )

            return False

        # OPEN:
        # We expect the requested open position to be reached.
        if not closing:

            if not result.reached_goal:
                self.get_logger().error(
                    'Gripper did not reach OPEN position'
                )
                return False

        # CLOSE:
        # Reached goal OR stalled against object are both acceptable
        # at this stage.
        else:

            if not (
                result.reached_goal
                or result.stalled
            ):

                self.get_logger().error(
                    'Gripper neither closed nor contacted object'
                )

                return False

        return True

