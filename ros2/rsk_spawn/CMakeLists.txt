cmake_minimum_required(VERSION 3.5)
project(rsk_spawn)
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(gazebo_ros REQUIRED)
ament_package()
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
