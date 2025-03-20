cmake_minimum_required(VERSION 3.5)
project(vff_avoidance)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

add_executable(avoidance_vff_main
  src/avoidance_vff_main.cpp
  src/vff_avoidance/AvoidanceNode.cpp
)

ament_target_dependencies(avoidance_vff_main
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  yaml-cpp
)

install(TARGETS
  avoidance_vff_main
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
