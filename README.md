# Ros2-Doosan

Dự án điều khiển robot Doosan sử dụng ROS2 và Python, tập trung vào việc điều khiển robot thông qua các lệnh di chuyển và tương tác với Modbus.

## Yêu cầu hệ thống

- Ubuntu 22.04
- ROS2 Humble
- Python 3.8+
- Doosan Robot Controller

## Cài đặt

1. Clone repository:
```bash
git https://github.com/DoosanRobotics/doosan-robot2
cd Ros2-Doosan
```

2. Cấu hình ROS2:
```bash
source /opt/ros/foxy/setup.bash  # hoặc source /opt/ros/humble/setup.bash
```

## Cấu trúc dự án

### File chính

- `ros2_tsvison.py`: File chính để điều khiển robot, xử lý các lệnh di chuyển và tương tác với Modbus
- `movel.py`: Module điều khiển chuyển động tuyến tính của robot
- `config_tcp.py`: Cấu hình TCP (Tool Center Point)

### Module điều khiển

- `get_current_pose.py`: Lấy vị trí hiện tại của robot
- `get_current_tcp.py`: Lấy thông tin TCP hiện tại
- `robot_mode.py`: Quản lý chế độ hoạt động của robot
- `set_current_tcp.py`: Thiết lập TCP mới
- `set_output_modbus.py`: Điều khiển ngõ ra Modbus
- `set_output_crtbox.py`: Điều khiển ngõ ra Control Box

### Module Modbus

- `config_create_modbusTCP.py`: Tạo cấu hình Modbus TCP
- `delete_modbus.py`: Xóa cấu hình Modbus
- `get_modbus_input.py`: Đọc ngõ vào Modbus

## Hướng dẫn sử dụng

### 1. Khởi động hệ thống

```bash
chmod +x ros2.sh
./ros2.sh
```

### 2. Điều khiển robot

#### Di chuyển robot
```python
python movel.py
```
File này sẽ:
- Thực hiện chuỗi di chuyển theo các vị trí đã định nghĩa
- Điều khiển các ngõ ra Modbus
- Các vị trí mặc định bao gồm:
  - Home: [-136.8, 4.97, 196.52, 125.66, 176.35, 122.51]
  - ReadyToPick_Suction: [-451.75, -191.56, 286.9, 12.97, -176.25, 12.83]

#### Điều khiển thực tế

1. Lấy vị trí hiện tại:
```python
python get_current_pose.py
```

2. Thiết lập TCP:
```python
python set_current_tcp.py
```

### 3. Điều khiển Modbus

1. Tạo cấu hình Modbus:
```python
python config_create_modbusTCP.py
```

2. Đọc ngõ vào:
```python
python get_modbus_input.py
```

3. Điều khiển ngõ ra:
```python
python set_output_modbus.py
```

## Các tham số quan trọng

### Vận tốc và gia tốc
- Vận tốc tuyến tính: 100.0 mm/s
- Vận tốc góc: 100.0 deg/s
- Gia tốc tuyến tính: 100.0 mm/s²
- Gia tốc góc: 100.0 deg/s²

### Các vị trí đặc biệt
- Home: [-184.06, -25.00, 115.00, 1.48, 89.39, 3.73]
- ReadyToPick_Suction: [-425.65, -244.13, 286.81, 19.78, -176.26, -164.3]
- OutBin_Suction: [-468.06, -206.81, 294.7, 175.32, 177.32, -8.17]
- PreDrop_Suction: [-452.87, -275.41, 267.31, 118.0, -178.95, -71.72]
- Drop_Suction: [-475.62, -289.35, 157.48, 14.19, 178.31, -175.07]

## Lưu ý bảo mật

- Luôn kiểm tra vùng làm việc an toàn trước khi chạy robot
- Đảm bảo không có vật cản trong vùng làm việc
- Tuân thủ các quy định an toàn của nhà sản xuất

## Xử lý lỗi

1. Lỗi kết nối ROS2:
   - Kiểm tra ROS2 đã được cài đặt đúng cách
   - Kiểm tra kết nối mạng
   - Kiểm tra robot đã được bật và kết nối

2. Lỗi Modbus:
   - Kiểm tra địa chỉ IP và port
   - Kiểm tra cấu hình Modbus
   - Kiểm tra kết nối vật lý

## Đóng góp

Mọi đóng góp đều được hoan nghênh. Vui lòng tạo issue hoặc pull request để đóng góp.

## Giấy phép

Dự án này được phân phối dưới giấy phép MIT. Xem file `LICENSE` để biết thêm chi tiết.