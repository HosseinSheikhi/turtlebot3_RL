import sys

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from service.service_member import MinimalService


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()
        self.grp_timer = MutuallyExclusiveCallbackGroup()
        self.create_timer(1.0, self.send_request, callback_group=self.grp_timer)

    def send_request(self):
        self.req.a = 2
        self.req.b = 3
        self.future = self.cli.call_async(self.req)

        rclpy.spin_until_future_complete(self, self.future)

        try:
            response = self.future.result()
        except Exception as e:
            self.get_logger().info(
                'Service call failed %r' % (e,))
        else:
            self.get_logger().info(
                'Result of add_two_ints: for %d + %d = %d' %
                (self.req.a, self.req.b, response.sum))


def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClientAsync()

    while rclpy.ok():
        rclpy.spin_once(minimal_client)

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
