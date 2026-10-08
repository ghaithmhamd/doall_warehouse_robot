#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from example_interfaces.action import Fibonacci

import time


class SimpleActionServer(Node):

    def __init__(self):
        super().__init__('simple_action_server')

        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.get_logger().info("Action Server Started")

    # ================= GOAL CALLBACK =================
    def goal_callback(self, goal_request):
        self.get_logger().info(f"Received goal request: {goal_request.order}")

        if goal_request.order <= 0:
            self.get_logger().info("Rejecting goal")
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    # ================= CANCEL CALLBACK =================
    def cancel_callback(self, goal_handle):
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    # ================= EXECUTE CALLBACK =================
    async def execute_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Executing goal...")

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(2, goal_handle.request.order):

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("Goal canceled")
                return Fibonacci.Result()

            next_number = feedback_msg.sequence[i-1] + feedback_msg.sequence[i-2]
            feedback_msg.sequence.append(next_number)

            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f"Feedback: {feedback_msg.sequence}")

            time.sleep(1)

        goal_handle.succeed()

        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence

        self.get_logger().info("Goal succeeded")
        return result


def main(args=None):
    rclpy.init(args=args)

    node = SimpleActionServer()
    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
