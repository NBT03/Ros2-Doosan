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
import socket

#define pose
# home = [-136.8, 4.97, 196.52, 125.66, 176.35, 122.51],
# ReadyToPick_Suction = [-451.75, -191.56, 286.9, 12.97, -176.25, 12.83],   
# OutBin_Suction = [-451.12, -314.32, 288.31, 12.89, -176.28, 12.75],
# PreDrop_Suction = [-453.41, -419.75, 239.07, 13.01, -176.27, 12.86],
# Drop_Suction = [-459.15, -421.0, 146.0, 13.02, -176.27, 12.88],
# home = [-22.87, -164.01, 193.95, 47.18, 178.4, -128.69], 
home = [[-184.06, -25.00, 115.00, 1.48, 89.39, 3.73],]
ReadyToPick_Suction = [-425.65, -244.13, 286.81, 19.78, -176.26, -164.3], 
OutBin_Suction = [-468.06, -206.81, 294.7, 175.32, 177.32, -8.17],
PreDrop_Suction = [-452.87, -275.41, 267.31, 118.0, -178.95, -71.72],
Drop_Suction = [-475.62, -289.35, 157.48, 14.19, 178.31, -175.07], 

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
            self.req.vel = 100.0  # Tốc độ di chuyển
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

            time.sleep(1.0)
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
            self.req.vel = [1000.0, 1000.0]
            self.req.acc = [1000.0, 1000.0]
            self.req.time = 0.0
            self.req.radius = 120.0
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

class SetDigitalOutputClient(Node):

    def __init__(self):
        super().__init__('set_digital_output_client')
        self.client = self.create_client(SetCtrlBoxDigitalOutput, '/dsr01/io/set_ctrl_box_digital_output')

        # Wait for the service to be available
        while not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Waiting for service /dsr01/io/set_ctrl_box_digital_output...')

    def send_request(self, index, value):
        request = SetCtrlBoxDigitalOutput.Request()
        request.index = index
        request.value = value
        # Send the request and wait for the response
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f"Set output index {index} to value {value} successfully.")
        else:
            self.get_logger().error(f"Failed to set output index {index} to value {value}.")

def main(args=None):
    HOST = '0.0.0.0'  # Lắng nghe trên tất cả các IP của máy
    PORT = 12345  # Cổng kết nối

    # Tạo socket TCP
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((HOST, PORT))  # Gắn kết socket với địa chỉ và cổng
    server_socket.listen(10)
    print(f"🔵 Server đang chạy trên {HOST}:{PORT}...")
    rclpy.init(args=args)
    movel = MoveLineController()
    # cfg_modbus()
    set_robot_mode = SetRobotModeClient()
    set_robot_mode.send_request(0)
    time.sleep(1)
    tcp_name = 'Suction'
    creat_tcp = CreateTcpClient()
    TCP = [-7.448, -92.785, 183.447, 0.0, 0.0, 0.0]  # Replace with the desired positions
    creat_tcp.send_request(tcp_name, TCP)
    set_current_tcp = SetCurrentTcpClient()
    set_current_tcp.send_request(tcp_name)
    time.sleep(1)
    set_robot_mode.send_request(1)
    movej = RobotTrajectoryController()
    modbus_output_setter = ModbusOutputSetter()
    digital_output_client = SetDigitalOutputClient()
    # modbus_output_setter.set_modbus_output('gripper_signal_1', 2304)
    digital_output_client.send_request(6, 0)
    digital_output_client.send_request(7, 0)
    # movel.send_trajectory(home)
    movej.send_trajectory(home)
    digital_output_client.destroy_node()
    conn, addr = server_socket.accept()  # Chấp nhận kết nối
    print(f"🟢 Nhận kết nối từ {addr}")
    while True:
        digital_output_client = SetDigitalOutputClient()
        print("1")
        data = conn.recv(1024).decode()  # Nhận dữ liệu từ client
        Pick = data.split(",")
        pickType = str(Pick[0])
        if not data:
            break
        print(f"📩 Dữ liệu nhận được: {data}")
        print(Pick)
        print(pickType)
        if pickType == "s":
            PrePick = [float(num) for num in Pick[1:7]],
            TarPick = [float(num) for num in Pick[7:]],
            print("PrePick", PrePick)
            print("TarPick", TarPick)
            movel.send_trajectory(ReadyToPick_Suction)
            movel.send_trajectory(PrePick)
            digital_output_client.send_request(6, 1)
            digital_output_client.send_request(7, 1)

            movel.send_trajectory(TarPick)
            # begin_blend(radius = 120)
            movel.send_trajectory(PrePick)
            movel.send_trajectory(ReadyToPick_Suction)
            movel.send_trajectory(OutBin_Suction)
            conn.send("finish".encode())
            movel.send_trajectory(PreDrop_Suction)
            movel.send_trajectory(Drop_Suction)
            digital_output_client.send_request(6, 0)
            digital_output_client.send_request(7, 0)
    # home_pose = [
    #     [-94.06, -13.65, 101.87, 1.48, 89.39, 3.73],
    # ]
    # movej.send_trajectory(home_pose)
    # modbus_output_setter.set_modbus_output('gripper_signal_3', 0)  # Ví dụ: output_3 = 1
    # modbus_output_setter.set_modbus_output('gripper_signal_5', 25000)  # Ví dụ: output_2 = 0
    # movej.send_trajectory(bf_grasp)
    # time.sleep(1)
    # movej.send_trajectory(grasp)
    # time.sleep(1)
    # modbus_output_setter.set_modbus_output('gripper_signal_3', 100)  # Ví dụ: output_3 = 1
    # modbus_output_setter.set_modbus_output('gripper_signal_5', 25000)  # Ví dụ: output_2 = 0
    # time.sleep(1)
    # movej.send_trajectory(bf_grasp)
    # time.sleep(1)
    # movej.send_trajectory(out)
    # time.sleep(1)
    # modbus_output_setter.set_modbus_output('gripper_signal_3', 0)  # Ví dụ: output_3 = 1
    # modbus_output_setter.set_modbus_output('gripper_signal_5', 25000)  # Ví dụ: output_2 = 0
    # movej.send_trajectory(home_pose)
    # # Danh sách các tập hợp góc khớp cần di chuyển
    # rclpy.shutdown()

if __name__ == '__main__':
    main()


