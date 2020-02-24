cmake_minimum_required(VERSION 3.5)
project(allen)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

message(status ${CMAKE_MODULE_PATH})
message(status ${CMAKE_ROOT})
#find_path(CURL_INCLUDE_DIR NAMES curl/curl.h)
message(status ${CURL_INCLUDE_DIR})


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(ros2_talker src/ros2_talker.cpp)
ament_target_dependencies(ros2_talker rclcpp std_msgs)

add_executable(ros2_listerner src/ros2_listerner.cpp)
ament_target_dependencies(ros2_listerner rclcpp std_msgs)

install(TARGETS
  ros2_talker
  ros2_listerner
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
