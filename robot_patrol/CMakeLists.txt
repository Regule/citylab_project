cmake_minimum_required(VERSION 3.8)
project(robot_patrol)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(patrol_node src/patrol.cpp)
ament_target_dependencies(patrol_node rclcpp geometry_msgs sensor_msgs)

install(TARGETS
   patrol_node
   DESTINATION lib/${PROJECT_NAME}
)

 install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
include/
DESTINATION include
)

ament_package()
