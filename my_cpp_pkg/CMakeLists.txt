cmake_minimum_required(VERSION 3.8)
project(my_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies, new ones added to package.xml need to be added here too
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(my_robot_interfaces REQUIRED)

# CODE TO ADD TO COMPILE my_first_node.cpp
add_executable(cpp_node src/my_first_node.cpp)
# link dependencies
ament_target_dependencies(cpp_node rclcpp)
# Executable is created!

add_executable(number_publisher src/number_publisher.cpp)
ament_target_dependencies(number_publisher rclcpp example_interfaces)

add_executable(number_counter src/number_counter.cpp)
ament_target_dependencies(number_counter rclcpp example_interfaces)

add_executable(add_ints_server src/add_two_ints_server.cpp)
ament_target_dependencies(add_ints_server rclcpp example_interfaces)

add_executable(add_ints_client_no_oop src/add_two_ints_client_no_oop.cpp)
ament_target_dependencies(add_ints_client_no_oop rclcpp example_interfaces)

add_executable(add_ints_client src/add_two_ints_client.cpp)
ament_target_dependencies(add_ints_client rclcpp example_interfaces)

add_executable(hardware_status_pub src/hw_status_publisher.cpp)
ament_target_dependencies(hardware_status_pub rclcpp my_robot_interfaces)

add_executable(led_panel_server src/led_panel_server.cpp)
ament_target_dependencies(led_panel_server rclcpp my_robot_interfaces)

add_executable(battery_client src/battery_client.cpp)
ament_target_dependencies(battery_client rclcpp my_robot_interfaces)

add_executable(robot_news_station src/robot_news_station.cpp)
ament_target_dependencies(robot_news_station rclcpp example_interfaces)

add_executable(smartphone src/smartphone.cpp)
ament_target_dependencies(smartphone rclcpp example_interfaces)

# The node needs to be installed inside the workspace:
install(TARGETS
  cpp_node
  number_publisher
  number_counter
  add_ints_server
  add_ints_client
  add_ints_client_no_oop
  hardware_status_pub
  led_panel_server
  battery_client
  robot_news_station
  smartphone
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
