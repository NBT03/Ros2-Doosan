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
            self.req.vel = 50.0  # Tốc độ di chuyển
            self.req.acc = 50.0  # Gia tốc
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
    modbus_output_setter = ModbusOutputSetter()
    hom_pose = [[-180.25440979003906, -34.360267639160156, 120.13651275634766, -2.418900489807129, 91.78060913085938, 4.818256378173828],]
    pre_pick = [[-188.56085205078125, 3.511002540588379, 94.67302703857422, 0.09725085645914078, 79.82866668701172, -11.41978645324707],]
    pick = [[-189.0299072265625, 20.919200897216797, 103.45702362060547, -0.8632915019989014, 60.25026321411133, -82.31758117675781],]
    pos1 = [[-178.12815856933594, 1.3617578744888306, 88.70342254638672, 7.373632431030273, 77.79966735839844, -82.3172836303711],]
    pos2 =[[-170.2053985595703, 1.2281392812728882, 82.86988067626953, 10.333566665649414, 77.80470275878906, -82.3172836303711],]
    pos3 = [[-165.0592041015625, -1.3221592903137207, 69.28319549560547, 10.347925186157227, 78.56644439697266, -82.3172836303711],]
    pos4 = [[-155.70631408691406, -1.076535701751709, 54.59389114379883, 8.420475006103516, 78.67391204833984, -82.3172836303711],]
    pos5 = [[-137.49000549316406, 5.589290618896484, 47.449195861816406, 8.595293998718262, 79.32552337646484, -82.3172836303711],]
    pos6 = [[-114.55667877197266, 7.204751491546631, 42.75711441040039, 10.525260925292969, 80.58538055419922, -82.3172836303711],]
    pos7 = [[-96.35743713378906, 11.877551078796387, 42.86037826538086, 10.5260009765625, 82.20330810546875, -82.3172836303711],]
    pos8 = [[-76.66243743896484, 23.240652084350586, 70.67701721191406, -2.153193235397339, 81.99858856201172, -82.3172836303711],]
    movej = RobotTrajectoryController()
    movej.send_trajectory(hom_pose)
    movej.send_trajectory(pre_pick)
    movej.send_trajectory(pick)
    modbus_output_setter.set_modbus_output('gripper_signal_3', 150)
    time.sleep(1)  # Ví dụ: output_3 = 1
    movej.send_trajectory(pre_pick)
    movej.send_trajectory(pos1)
    movej.send_trajectory(pos2)
    movej.send_trajectory(pos3)
    movej.send_trajectory(pos4)
    movej.send_trajectory(pos5)
    movej.send_trajectory(pos6)
    movej.send_trajectory(pos7)
    movej.send_trajectory(pos8)
    modbus_output_setter.set_modbus_output('gripper_signal_3', 0)  # Ví dụ: output_3 = 1

    # time.sleep(2)
    # movej.send_trajectory(test_pose)
    # time.sleep(2)    
if __name__ == '__main__':
    main()
