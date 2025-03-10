import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import ConfigCreateTcp


class CreateTcpClient(Node):
    def __init__(self):
        super().__init__('create_tcp_client')
        self.client = self.create_client(ConfigCreateTcp, '/dsr01/tcp/config_create_tcp')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.get_logger().info('Service available, ready to send request.')

    def send_request(self, name, positions):
        request = ConfigCreateTcp.Request()
        request.name = name
        request.pos = positions

        self.future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        if self.future.result() is not None:
            if self.future.result().success:
                self.get_logger().info(f"TCP '{name}' created successfully!")
            else:
                self.get_logger().error(f"Failed to create TCP '{name}'. Check robot logs for details.")
        else:
            self.get_logger().error('Service call failed')


def main(args=None):
    rclpy.init(args=args)

    client = CreateTcpClient()

    # Example data for the service
    name = 'Tool'  # Provide the TCP name here
    positions = [5.670, -80.975, 187.318, 0.0, 0.0, 0.0]  # Replace with the desired positions

    client.send_request(name, positions)

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
