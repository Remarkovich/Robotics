# Robotics

ros2 pkg create --build-type ament_python --license Apache-2.0 'имя пакета' --dependencies rclpy example_interfaces
colcon build --packages-select service_full_name 'имя пакета' - Сборка пакета
source install/setup.bash
