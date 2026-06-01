cmake_minimum_required(VERSION 3.8)
project(ekf_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 의존성
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(gkusv_driver_interface REQUIRED)

# Eigen3
find_package(Eigen3 REQUIRED)

# 실행파일
add_executable(ekf_localization_node src/ekf_localization_node.cpp)

target_include_directories(ekf_localization_node PRIVATE
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(ekf_localization_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  gkusv_driver_interface
)

# 설치
install(TARGETS
  ekf_localization_node
  DESTINATION lib/${PROJECT_NAME}
)

find_package(visualization_msgs REQUIRED)

add_executable(ekf_visualizer_node src/ekf_visualizer_node.cpp)
ament_target_dependencies(ekf_visualizer_node
  rclcpp
  geometry_msgs
  nav_msgs
  visualization_msgs
  gkusv_driver_interface
)

install(TARGETS
  ekf_visualizer_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
