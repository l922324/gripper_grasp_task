#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from control_msgs.action import GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint

from tf2_ros import Buffer, TransformListener


ARM_JOINTS = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

PRE_GRASP_JOINTS = [
    -0.3353270000000017,
    -0.9733189999692432,
    0.8891590000844279,
    -1.4539729999584443,
    -1.5119640000026302,
    -0.31415926535897104,
]

GRASP_JOINTS = [
    -0.3353270000000011,
    -0.7533189999665528,
    0.8891590005161732,
    -1.453972999966789,
    -1.5119640000024814,
    -0.3141592653589627,
]


GRIPPER_OPEN = 0.10
GRIPPER_CLOSE = 0.75


class FixedPickLift(Node):

    def __init__(self):
        super().__init__('fixed_pick_lift')

        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )

        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            '/robotiq_gripper_controller/gripper_cmd'
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

    # ========================================================
    # Arm
    # ========================================================

    def move_arm(self, positions, duration=3.0, timeout=20.0):

        self.get_logger().info(
            f'Moving arm to {positions}'
        )

        if not self.arm_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                'Arm action server unavailable'
            )
            return False

        goal = FollowJointTrajectory.Goal()

        goal.trajectory.joint_names = ARM_JOINTS

        point = JointTrajectoryPoint()

        point.positions = positions

        point.time_from_start = Duration(
            seconds=duration
        ).to_msg()

        goal.trajectory.points = [point]

        send_future = self.arm_client.send_goal_async(goal)

        rclpy.spin_until_future_complete(
            self,
            send_future,
            timeout_sec=5.0
        )

        if not send_future.done():
            self.get_logger().error(
                'Timeout sending arm goal'
            )
            return False

        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error(
                'Arm goal rejected'
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
                'Arm movement timeout - cancelling'
            )

            cancel_future = goal_handle.cancel_goal_async()

            rclpy.spin_until_future_complete(
                self,
                cancel_future,
                timeout_sec=2.0
            )

            return False

        result = result_future.result()

        if result.status != GoalStatus.STATUS_SUCCEEDED:

            self.get_logger().error(
                f'Arm movement failed. Status={result.status}'
            )

            return False

        self.get_logger().info(
            'Arm movement succeeded'
        )

        return True

    # ========================================================
    # Gripper
    # ========================================================

    def move_gripper(
        self,
        position,
        closing=False,
        timeout=4.0
    ):

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
        goal.command.max_effort = 50.0

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

    # ========================================================
    # TF
    # ========================================================

    def get_tcp_z(self):

        try:

            transform = self.tf_buffer.lookup_transform(
                'world',
                'tool0',
                rclpy.time.Time(),
                timeout=Duration(seconds=2.0)
            )

            return transform.transform.translation.z

        except Exception as exc:

            self.get_logger().error(
                f'Unable to read TCP transform: {exc}'
            )

            return None

    # ========================================================
    # Primitive
    # ========================================================

    def run(self):

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            'Starting FIXED PICK + LIFT primitive'
        )

        self.get_logger().info(
            '========================================'
        )

        # 1. PRE-GRASP

        if not self.move_arm(
            PRE_GRASP_JOINTS,
            duration=4.0
        ):
            return self.fail('PRE_GRASP_FAILED')

        time.sleep(0.5)

        # 2. OPEN

        if not self.move_gripper(
            GRIPPER_OPEN,
            closing=False
        ):
            return self.fail('GRIPPER_OPEN_FAILED')

        time.sleep(0.5)

        # 3. APPROACH / GRASP

        if not self.move_arm(
            GRASP_JOINTS,
            duration=3.0
        ):
            return self.fail('APPROACH_FAILED')

        time.sleep(0.5)

        # 4. CLOSE

        if not self.move_gripper(
            GRIPPER_CLOSE,
            closing=True
        ):
            return self.fail('GRIPPER_CLOSE_FAILED')

        time.sleep(0.5)

        z_before = self.get_tcp_z()

        if z_before is None:
            return self.fail('TCP_READ_FAILED')

        self.get_logger().info(
            f'TCP before lift: Z={z_before:.4f} m'
        )

        # 5. LIFT
        #
        # PRE_GRASP is deliberately 100 mm above GRASP.

        if not self.move_arm(
            PRE_GRASP_JOINTS,
            duration=1.0
        ):
            return self.fail('LIFT_FAILED')

        time.sleep(0.5)

        z_after = self.get_tcp_z()

        if z_after is None:
            return self.fail('TCP_READ_FAILED')

        delta_z = z_after - z_before

        self.get_logger().info(
            f'TCP after lift: Z={z_after:.4f} m'
        )

        self.get_logger().info(
            f'TCP lift distance: {delta_z * 1000:.1f} mm'
        )

        # 6. Validate 100 mm +/- 10 mm

        if not 0.090 <= delta_z <= 0.110:

            return self.fail(
                f'LIFT_DISTANCE_OUT_OF_RANGE: '
                f'{delta_z * 1000:.1f} mm'
            )

        # 7. HOLD

        self.get_logger().info(
            'Holding object for 3 seconds...'
        )

        time.sleep(3.0)

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            f'Lift = {delta_z * 1000:.1f} mm'
        )

        self.get_logger().info(
            'Hold = 3.0 seconds'
        )

        self.get_logger().info(
            '========================================'
        )

        # 8. RELEASE / APPROACH

        if not self.move_arm(
            GRASP_JOINTS,
            duration=3.0
        ):
            return self.fail('APPROACH_FAILED')

        time.sleep(0.5)

        # 9. OPEN

        if not self.move_gripper(
            GRIPPER_OPEN,
            closing=False
        ):
            return self.fail('GRIPPER_OPEN_FAILED')

        time.sleep(0.5)

        # 10. RELEASE / APPROACH

        if not self.move_arm(
            PRE_GRASP_JOINTS,
            duration=3.0
        ):
            return self.fail('APPROACH_FAILED')

        time.sleep(0.5)

        # PASS

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            'PICKUP SIM PASS'
        )

        self.get_logger().info(
            '========================================'
        )

        return True

    def fail(self, reason):

        self.get_logger().error(
            '========================================'
        )

        self.get_logger().error(
            f'PICKUP SIM FAILED: {reason}'
        )

        self.get_logger().error(
            '========================================'
        )

        return False


def main(args=None):

    rclpy.init(args=args)

    node = FixedPickLift()

    # Give TF a moment to populate.
    start = time.time()

    while (
        rclpy.ok()
        and time.time() - start < 1.0
    ):
        rclpy.spin_once(
            node,
            timeout_sec=0.1
        )

    success = node.run()

    node.destroy_node()

    rclpy.shutdown()

    raise SystemExit(
        0 if success else 1
    )


if __name__ == '__main__':
    main()
