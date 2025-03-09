import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import MoveLine  # Import service tương ứng
import time

class MoveLineController(Node):
    def __init__(self):
        super().__init__('move_line_controller')
        self.client = self.create_client(MoveLine, '/dsr01/motion/move_line')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.req = MoveLine.Request()

    def send_trajectory(self, trajectory):
        for position in trajectory:
            self.req.pos = position
            self.req.vel = [10.0, 10.0]  # Tốc độ di chuyển (tuyến tính, góc)
            self.req.acc = [10.0, 10.0]  # Gia tốc (tuyến tính, góc)
            self.req.time = 0.0
            self.req.radius = 0.0
            self.req.ref = 0
            self.req.mode = 0
            self.req.blend_type = 1
            self.req.sync_type = 0

            future = self.client.call_async(self.req)
            rclpy.spin_until_future_complete(self, future)

            if future.result().success:
                self.get_logger().info('✅ Moved to: %s' % position)
            else:
                self.get_logger().error('❌ Failed to move to: %s' % position)

            time.sleep(1.0)

def main():
    rclpy.init()

    trajectory = [
        # [-5.82,-285.69,199.75,117.32,-177.08,-144.85],
        [82.43, -528.97, 256.45, 42.68, -176.82, 134.2],
    ]

    move_line_node = MoveLineController()
    move_line_node.send_trajectory(trajectory)

    move_line_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()