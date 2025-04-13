import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import MoveLine  # Import service tương ứng
from dsr_msgs2.srv import SetModbusOutput
import time

class ModbusOutputSetter(Node):

    def __init__(self):
        super().__init__('modbus_output_setter')
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
            self.req.vel = [100.0, 100.0]  # Tốc độ di chuyển (tuyến tính, góc)
            self.req.acc = [100.0, 100.0]  # Gia tốc (tuyến tính, góc)
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
    [-136.8,    4.97,   196.52, 125.66, 176.35, 122.51],  # Home
    [-451.75, -191.56,  286.9,   12.97, -176.25,  12.83],  # ReadyToPick_Suction
    # [-451.12, -314.32,  288.31,  12.89, -176.28,  12.75],  # Outpin ss
    # [-459.15, -421.0,   146.0,   13.02, -176.27,  12.88],  # Drop ss
    # [-453.41, -419.75,  239.07,  13.01, -176.27,  12.86],  # PreDrop Suction
    # [-458.6,  -221.51,  290.89,  12.89, -176.28,  12.75],  # Outbin
    # [-460.89, -326.94,  241.63,  13.0,  -176.27,  12.86],  # PreDrop
    # [-466.63, -328.18,  148.55,  13.01, -176.27,  12.87],  # Drop
]
    move_line_node = MoveLineController()
    move_line_node.send_trajectory(trajectory)
    modbus_output_setter = ModbusOutputSetter()

    # Set giá trị output cho 3 chân Modbus
    # modbus_output_setter.set_modbus_output('gripper_signal_1', 2304)  # Ví dụ: output_1 = 1
    modbus_output_setter.set_modbus_output('gripper_signal_3', 0)  # Ví dụ: output_3 = 1
    modbus_output_setter.set_modbus_output('gripper_signal_5', 25000)  # Ví dụ: output_2 = 0
    time.sleep(0.5)
    modbus_output_setter.set_modbus_output('gripper_signal_3', 150)
    modbus_output_setter.set_modbus_output('gripper_signal_5', 25000)  # Ví dụ: output_3 = 1
    move_line_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()