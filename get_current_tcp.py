import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import GetCurrentTcp


class GetCurrentTcpClient(Node):
    def __init__(self):
        super().__init__('get_current_tcp_client')
        self.client = self.create_client(GetCurrentTcp, '/dsr01/tcp/get_current_tcp')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.get_logger().info('Service available, ready to send request.')

    def send_request(self):
        request = GetCurrentTcp.Request()

        self.future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)

        if self.future.result() is not None:
            response = self.future.result()
            if response.success:
                self.get_logger().info(f"Current TCP: {response.info}")
            else:
                self.get_logger().error("Failed to get current TCP")
        else:
            self.get_logger().error('Service call failed')


def main(args=None):
    rclpy.init(args=args)

    client = GetCurrentTcpClient()
    client.send_request()

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
