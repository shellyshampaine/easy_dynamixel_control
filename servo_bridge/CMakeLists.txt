cmake_minimum_required(VERSION 3.8)
project(servo_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ---- Messages ----------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SetpointReached.msg"
  DEPENDENCIES std_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ---- Bridge node -------------------------------------------------------------
add_executable(servo_bridge_node src/servo_bridge_node.cpp)
target_compile_features(servo_bridge_node PUBLIC cxx_std_17)
ament_target_dependencies(servo_bridge_node rclcpp sensor_msgs std_msgs)
target_link_libraries(servo_bridge_node "${cpp_typesupport_target}")

# ---- Plain serial terminal (debug tool, no ROS) ------------------------------
add_executable(talker src/talker.cpp)

install(TARGETS servo_bridge_node talker
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
