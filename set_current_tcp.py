import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import SetCurrentTcp


class SetCurrentTcpClient(Node):
    def __init__(self):
        super().__init__('set_current_tcp_client')
        self.client = self.create_client(SetCurrentTcp, '/dsr01/tcp/set_current_tcp')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.get_logger().info('Service available, ready to send request.')

    def send_request(self, tcp_name):
        request = SetCurrentTcp.Request()
        request.name = tcp_name

        self.future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)

        if self.future.result() is not None:
            if self.future.result().success:
                self.get_logger().info(f"Successfully set current TCP to '{tcp_name}'")
            else:
                self.get_logger().error(f"Failed to set current TCP to '{tcp_name}'")
        else:
            self.get_logger().error('Service call failed')


def main(args=None):
    rclpy.init(args=args)

    client = SetCurrentTcpClient()

    # Replace 'Tool_1' with the desired TCP name
    tcp_name = 'Tool'
    client.send_request(tcp_name)

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
