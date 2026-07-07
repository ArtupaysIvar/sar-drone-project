cmake_minimum_required(VERSION 3.10)
project(gps_approx_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pixel_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)  
find_package(geometry_msgs REQUIRED)  
find_package(Eigen3 REQUIRED)



add_executable(calc_gps_node src/CalcGPSNode.cpp)

ament_target_dependencies(calc_gps_node
  rclcpp
  pixel_msgs
  geometry_msgs
  sensor_msgs
  px4_msgs
  # Eigen3
  # Ceres
)

target_link_libraries(calc_gps_node
  Eigen3::Eigen
)
# punya orang:
# target_include_directories(control_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)

# target_compile_features(control_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


install(TARGETS
  calc_gps_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()


# cmake_minimum_required(VERSION 3.10)
# project(gps_approx_pkg)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # find dependencies
# find_package(ament_cmake REQUIRED)
# # find rclcpp for ROS 2 C++ APIs
# find_package(rclcpp REQUIRED)
# # find_package(pixel_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# # uncomment the following section in order to fill in
# # further dependencies manually.
# # find_package(<dependency> REQUIRED)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()

# # build the node
# add_executable(calc_gps_node src/CalcGPSNode.cpp)
# ament_target_dependencies(calc_gps_node rclcpp)

# install(TARGETS
#   calc_gps_node
#   DESTINATION lib/${PROJECT_NAME}
# )