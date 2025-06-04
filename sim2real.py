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
            self.req.vel =100.0  # Tốc độ di chuyển
            self.req.acc = 100.0  # Gia tốc
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
def main(args=None):
    rclpy.init(args=args)
    modbus_output_setter = ModbusOutputSetter()
    modbus_output_setter.set_modbus_output('gripper_signal_3', 0)  # Ví dụ: output_3 = 1
    home = [[-184.06, -25.00, 115.00, 1.48, 89.39, 3.73],]
    target = [[-84.06, 30.00, 60.84, 1.48, 89.39, 3.73],]
    pose = [[-184.06000000000003, -13.65, 101.86999999999999, 1.4800000000000002, 89.39, 3.7300000000000004], [-165.2664052902455, -27.21972201980697, 98.8729718714132, 13.575196232640698, 85.29103291964726, -6.819291400759887], [-147.25551449566, -23.590523462734208, 80.54016079575445, 2.5398979758249984, 80.62098084407161, -8.679173703629518], [-122.46880124523675, -21.92307396748091, 81.80955241874327, 9.452384658148935, 82.15729111477866, 3.6409711596141046], [-107.2176100965166, -2.535160307288452, 73.32951800551001, 14.704920966508686, 75.28309885340522, 11.731516220762597], [-87.59229133582375, 5.1876157439113735, 59.373940254882555, 6.59800568270297, 78.47730591487857, 1.4736413637090822], [-75.7740929322138, 28.548902712145956, 60.782506262348626, 1.7793152164959494, 88.75179527825041, 3.5980418671678294], [-75.04, 29.999999999999996, 60.870000000000005, 1.4800000000000002, 89.39, 3.7300000000000004]]
    ready_grip = [[-189.8553009033203, 7.76143217086792, 97.4365005493164, -4.070281982421875, 70.46244812011719, -85.6170883178711],]
    grip = [[-190.90371704101562, 23.409793853759766, 104.26335144042969, -0.13722233474254608, 55.18095397949219, -85.61634826660156],]
    movej = RobotTrajectoryController()
    movej.send_trajectory(home)
    # movej.send_trajectory(ready_grip)
    # movej.send_trajectory(grip)
    # modbus_output_setter.set_modbus_output('gripper_signal_3', 150)
    # time.sleep(1)
    # movej.send_trajectory(ready_grip)
    # movej.send_trajectory(pose)
    # modbus_output_setter.set_modbus_output('gripper_signal_3', 0)  # Ví dụ: output_3 = 1

    # time.sleep(2)
    # movej.send_trajectory(test_pose)
    # time.sleep(2)    
if __name__ == '__main__':
    main()
