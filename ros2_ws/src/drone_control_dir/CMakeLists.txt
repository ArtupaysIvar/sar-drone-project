cmake_minimum_required(VERSION 3.8)
project(drone_control_dir)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(offboard_control1
  src/offboard_control1.cpp
)
add_executable(offboard_control2
  src/offboard_control2.cpp
)
add_executable(offboard_control3
  src/offboard_control3.cpp
)

ament_target_dependencies(
  offboard_control1
  rclcpp
  px4_msgs
  std_msgs
  sensor_msgs
)

ament_target_dependencies(
  offboard_control2
  rclcpp
  px4_msgs
  std_msgs
  sensor_msgs
)

ament_target_dependencies(
  offboard_control3
  rclcpp
  px4_msgs
  std_msgs
  sensor_msgs
)

target_link_libraries(offboard_control1 Eigen3::Eigen)
target_link_libraries(offboard_control2 Eigen3::Eigen)
target_link_libraries(offboard_control3 Eigen3::Eigen)

install(TARGETS
  offboard_control1
  offboard_control2
  offboard_control3
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
