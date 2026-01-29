cmake_minimum_required(VERSION 3.8)
project(my_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_executable(test_node src/my_first_node.cpp)
ament_target_dependencies(test_node rclcpp)

add_executable(number_publisher src/number_publisher.cpp)
ament_target_dependencies(number_publisher rclcpp example_interfaces std_msgs)

add_executable(number_counter src src/number_counter.cpp)
ament_target_dependencies(number_counter rclcpp example_interfaces)

add_executable(simple_lifecycle_node src src/simple_lifecycle_node.cpp)
ament_target_dependencies(simple_lifecycle_node rclcpp rclcpp_lifecycle std_msgs)

add_executable(simple_qos_publisher src/simple_qos_publisher.cpp)
ament_target_dependencies(simple_qos_publisher rclcpp std_msgs)

add_executable(simple_qos_subscriber src/simple_qos_subscriber.cpp)
ament_target_dependencies(simple_qos_subscriber rclcpp std_msgs)


install (TARGETS
  test_node
  number_publisher
  number_counter
  simple_lifecycle_node
  simple_qos_publisher
  simple_qos_subscriber
  DESTINATION lib/${PROJECT_NAME}/
)


ament_package()
