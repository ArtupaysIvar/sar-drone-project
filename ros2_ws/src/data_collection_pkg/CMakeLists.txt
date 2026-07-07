cmake_minimum_required(VERSION 3.10)
project(data_collection_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pixel_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)  
find_package(geometry_msgs REQUIRED)  
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(data_collection_node src/DataCollectionNode.cpp)
ament_target_dependencies(data_collection_node
  rclcpp
  pixel_msgs
  geometry_msgs
  sensor_msgs
  px4_msgs
  std_msgs
  # Eigen3
  # Ceres
)

target_link_libraries(data_collection_node
  Eigen3::Eigen
  # Ceres::ceres
)

install(TARGETS
  data_collection_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
