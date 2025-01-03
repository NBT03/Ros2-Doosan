import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import MoveJoint
from config_create_modbusTCP import main as cfg_modbus
from dsr_msgs2.srv import SetModbusOutput
import time

class ModbusOutputSetter(Node):

    def __init__(self):
        super().__init__('modbus_output_setter')
        # Sửa lại tên dịch vụ, loại bỏ khoảng trắng thừa
        self.cli = self.create_client(SetModbusOutput, '/dsr01/modbus/set_modbus_output')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

    def set_modbus_output(self, name, value):
        req = SetModbusOutput.Request()
        req.name = name
        req.value = value

        self.get_logger().info(f'Setting modbus output: {name}, Value: {value}')
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Successfully set modbus output: {name} = {value}')
        else:
            self.get_logger().error('Failed to set modbus output')

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

            # Chờ giữa các lần di chuyển nếu cần
            time.sleep(1.0)


def main(args=None):
    cfg_modbus()
    rclpy.init(args=args)
    robot_controller = RobotTrajectoryController()
    modbus_output_setter = ModbusOutputSetter()
    modbus_output_setter.set_modbus_output('gripper_signal_1', 2304)  # Ví dụ: output_1 = 1
    time.sleep(1)
    home_pose = [
        [-94.06, -13.65, 101.87, 1.48, 89.39, 3.73],
    ]
    bf_grasp = [
        [-93.25875092, 31.32101822, 56.63997269, -4.13985443, 89.58004761 ,3.72981906],
    ]
    grasp = [
        [-96.45167542, 38.41414642, 70.43661499, -2.21640038, 68.62203979, 3.73855257],
    ]
    out = [
        [-59.81486893 , 45.58809662 , 45.90498734 ,- 2.63783145 , 87.94761658, 3.73825669],
    ]
    robot_controller.send_trajectory(home_pose)
    modbus_output_setter.set_modbus_output('gripper_signal_3', 0)  # Ví dụ: output_3 = 1
    modbus_output_setter.set_modbus_output('gripper_signal_5', 25000)  # Ví dụ: output_2 = 0
    robot_controller.send_trajectory(bf_grasp)
    time.sleep(1)
    robot_controller.send_trajectory(grasp)
    time.sleep(1)
    modbus_output_setter.set_modbus_output('gripper_signal_3', 100)  # Ví dụ: output_3 = 1
    modbus_output_setter.set_modbus_output('gripper_signal_5', 25000)  # Ví dụ: output_2 = 0
    time.sleep(1)
    robot_controller.send_trajectory(bf_grasp)
    time.sleep(1)
    robot_controller.send_trajectory(out)
    time.sleep(1)
    modbus_output_setter.set_modbus_output('gripper_signal_3', 0)  # Ví dụ: output_3 = 1
    modbus_output_setter.set_modbus_output('gripper_signal_5', 25000)  # Ví dụ: output_2 = 0
    robot_controller.send_trajectory(home_pose)
    # Danh sách các tập hợp góc khớp cần di chuyển
    rclpy.shutdown()

if __name__ == '__main__':
    main()


