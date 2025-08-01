import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import GetCurrentPose

class GetCurrentPoseClient(Node):
    def __init__(self):
        super().__init__('get_current_pose_client')
        self.client = self.create_client(GetCurrentPose, '/dsr01/system/get_current_pose')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

    def get_pose(self, space_type):
        request = GetCurrentPose.Request()
        request.space_type = space_type
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            if future.result().success:
                pos = future.result().pos
                print(f"[{', '.join(map(str, pos))}]")
            else:
                print("Failed to retrieve pose.")
        else:
            print("Service call failed!")

def main():
    rclpy.init()
    client = GetCurrentPoseClient()
    client.get_pose(0)  # 0 = Joint Space
    rclpy.shutdown()

if __name__ == '__main__':
    main()
