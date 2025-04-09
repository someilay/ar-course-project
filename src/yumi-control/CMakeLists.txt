cmake_minimum_required(VERSION 3.8)
project(yumi-control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# ROS2 Control dependencies
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)

# Find Pinocchio and dependencies
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(eigenpy REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Custom controller plugin library
add_library(
  yumi_id_controller
  SHARED
  src/yumi_id_controller.cpp
)
target_include_directories(
  yumi_id_controller
  PRIVATE
  include
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(
  yumi_id_controller
  pinocchio::pinocchio
  eigenpy::eigenpy
  ${EIGEN3_LIBRARIES}
)
ament_target_dependencies(
  yumi_id_controller
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)

pluginlib_export_plugin_description_file(controller_interface controller_plugins.xml)

install(
  TARGETS
  yumi_id_controller
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
