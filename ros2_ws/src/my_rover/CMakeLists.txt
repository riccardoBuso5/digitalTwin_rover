cmake_minimum_required(VERSION 3.10)
project(my_rover)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(rover_controller src/rover_controller.cpp)
ament_target_dependencies(rover_controller rclcpp geometry_msgs sensor_msgs)


install(PROGRAMS
  scripts/wokwi_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS rover_controller DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch urdf worlds DESTINATION share/${PROJECT_NAME})


ament_package()