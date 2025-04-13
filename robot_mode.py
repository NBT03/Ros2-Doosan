import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import SetRobotMode

class SetRobotModeClient(Node):
    def __init__(self):
        super().__init__('set_robot_mode_client')
        self.client = self.create_client(SetRobotMode, '/dsr01/system/set_robot_mode')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

    def send_request(self, robot_mode):
        request = SetRobotMode.Request()
        request.robot_mode = robot_mode
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            if future.result().success:
                self.get_logger().info(f'Successfully set robot mode to {robot_mode}')
            else:
                self.get_logger().error(f'Failed to set robot mode to {robot_mode}')
        else:
            self.get_logger().error('Service call failed!')

def main():
    rclpy.init()
    client = SetRobotModeClient()
    client.send_request(1)# Thay đổi mode thành AUTONOMOUS (1)
    rclpy.shutdown()
# ros2 service call /dsr01/system/get_robot_mode dsr_msgs2/srv/GetRobotMode {}\
# ros2 service call /dsr01/system/set_robot_mode dsr_msgs2/srv/SetRobotMode {}\
if __name__ == '__main__':
    main()
