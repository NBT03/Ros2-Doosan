import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import MoveJoint
from config_create_modbusTCP import main as cfg_modbus
from dsr_msgs2.srv import SetModbusOutput
from dsr_msgs2.srv import MoveLine
from dsr_msgs2.srv import SetCtrlBoxDigitalOutput
from dsr_msgs2.srv import ConfigCreateTcp
from dsr_msgs2.srv import SetRobotMode
from dsr_msgs2.srv import SetCurrentTcp
import time

class RobotTrajectoryController(Node):
    def __init__(self):
        super().__init__('robot_trajectory_controller')
        self.client = self.create_client(MoveJoint, '/dsr01/motion/move_joint')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.req = MoveJoint.Request()

    def send_trajectory(self, trajectory):
        for joint_angles in trajectory:
            self.req.pos = joint_angles
            self.req.vel = 30.0  # Tốc độ di chuyển
            self.req.acc = 20.0  # Gia tốc
            self.req.time = 0.0  # Thời gian thực hiện lệnh
            self.req.radius = 0.0  # Bán kính chuyển động tròn nếu cần
            self.req.mode = 0  # Chế độ điều khiển
            self.req.blend_type = 1
            self.req.sync_type = 0

            future = self.client.call_async(self.req)
            rclpy.spin_until_future_complete(self, future)

            if future.result().success:
                self.get_logger().info('Successfully moved to: %s' % joint_angles)
            else:
                self.get_logger().error('Failed to move to: %s' % joint_angles)

            time.sleep(1.0)
def main(args=None):
    rclpy.init(args=args)
    home_pose = [[-178.67871094 , -33.3122406 ,  126.7074585   , 1.46486592   ,83.64774323,
    9.28657913], ]
    test_pose = [[-162.65447998 ,  34.91273117  , 45.35361862  , -3.62561083  , 96.73845673,
            9.28672695], ]
    movej = RobotTrajectoryController()
    movej.send_trajectory(home_pose)
    # time.sleep(2)
    # movej.send_trajectory(test_pose)
    # time.sleep(2)    
if __name__ == '__main__':
    main()
