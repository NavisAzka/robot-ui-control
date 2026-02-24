cmake_minimum_required(VERSION 3.8)
project(slam_web_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(bridge_node src/bridge_node.cpp)

ament_target_dependencies(
  bridge_node
  rclcpp
  sensor_msgs
  nav_msgs
  std_srvs
)

target_link_libraries(bridge_node pthread)

install(TARGETS
  bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()