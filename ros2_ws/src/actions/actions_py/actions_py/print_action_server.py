import rclpy
from action_interfaces.action import Print
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node


class PrintServer(Node):
    def __init__(self):
        super().__init__("print_action_server")

        self._action_server = ActionServer(
            self,
            Print,
            "print_action",
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return Print.Result()

        goal_handle.succeed()
        result = Print.Result()

        # get goal requst
        self.get_logger().info(f"{goal_handle.request.text}")

        return result


def main(args=None):
    rclpy.init(args=args)
    minimal_action_server = PrintServer()

    rclpy.spin(minimal_action_server)

    minimal_action_server.destroy()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
