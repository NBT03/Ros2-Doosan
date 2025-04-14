#!/bin/bash

# Thiết lập môi trường ROS2 (thay bằng đường dẫn ROS2 của bạn nếu khác)
source /opt/ros/humble/setup.bash

# Thiết lập workspace ROS2 (nếu có workspace riêng)
 source ~/ros2_ws/install/setup.bash

# Chạy lệnh launch
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=192.168.137.100 model:=m0609
