import sys

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_srvs.srv import Empty

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.group = MutuallyExclusiveCallbackGroup()
        self.cli = self.create_client(AddTwoInts, 'add_two_ints', callback_group=self.group)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()
        self.create_timer(1.0, self.send_request, callback_group=self.group)

        self.srv = self.create_service(Empty, 'empty', self.empty_callback)

    def empty_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Empty received' )

        return response

    def send_request(self):
        self.req.a = int(sys.argv[1])
        self.req.b = int(sys.argv[2])
        self.future = self.cli.call_async(self.req)
        self.get_logger().info('Add sent' )

        rclpy.spin_until_future_complete(self, self.future)

        if self.future.done():
            response = self.future.result()
            self.get_logger().info(
                'Result of add_two_ints: for %d' %
                (response.sum))


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    rclpy.spin(minimal_client)
    #executor = MultiThreadedExecutor(num_threads=4)
    #executor.add_node(minimal_client)
    # minimal_client.send_request()
    #executor.spin()
    # minimal_client.send_request()

    # while rclpy.ok():
    #     rclpy.spin_once(minimal_client)
    #     if minimal_client.future.done():
    #         try:
    #             response = minimal_client.future.result()
    #         except Exception as e:
    #             minimal_client.get_logger().info(
    #                 'Service call failed %r' % (e,))
    #         else:
    #             minimal_client.get_logger().info(
    #                 'Result of add_two_ints: for %d + %d = %d' %
    #                 (minimal_client.req.a, minimal_client.req.b, response.sum))
    #         break

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
