cmake_minimum_required(VERSION 3.8)
project(robot_patrol)

# -------------------------------- SET COMPILE OPTIONS ----------------------------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



# ---------------------------------- FIND DEPENDENCIES ----------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ------------------------------- CUSTOM INTERFACES PROCESSING ------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetDirection.srv"
  "action/GoToPose.action"
  DEPENDENCIES sensor_msgs geometry_msgs
)

# Required if using interfaces in same package
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)


# ------------------------------------ LIBRARIES PROCESSING ---------------------------------------
add_library(${PROJECT_NAME}_lib src/simple_lidar.cpp src/utils.cpp src/naive_goto.cpp)
ament_target_dependencies(${PROJECT_NAME}_lib rclcpp sensor_msgs geometry_msgs nav_msgs tf2 tf2_geometry_msgs)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(
  TARGETS ${PROJECT_NAME}_lib 
  EXPORT ${PROJECT_NAME}_lib 
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  include/
  DESTINATION include
)


# ---------------------------------- EXECUTABLES PROCESSING ---------------------------------------
add_executable(patrol_node src/patrol.cpp)
ament_target_dependencies(patrol_node rclcpp geometry_msgs sensor_msgs)
target_link_libraries(patrol_node ${PROJECT_NAME}_lib )

add_executable(direction_service_node src/direction_service.cpp)
ament_target_dependencies(direction_service_node rclcpp sensor_msgs)
target_link_libraries(direction_service_node ${PROJECT_NAME}_lib "${cpp_typesupport_target}")

add_executable(direction_service_test src/test_service.cpp)
ament_target_dependencies(direction_service_test rclcpp sensor_msgs)
target_link_libraries(direction_service_test "${cpp_typesupport_target}")

add_executable(patrol_with_service_node src/patrol_with_service.cpp)
ament_target_dependencies(patrol_with_service_node rclcpp geometry_msgs sensor_msgs)
target_link_libraries(patrol_with_service_node ${PROJECT_NAME}_lib "${cpp_typesupport_target}")

add_executable(go_to_pose_action_node src/go_to_pose_action.cpp)
ament_target_dependencies(go_to_pose_action_node rclcpp rclcpp_action sensor_msgs geometry_msgs nav_msgs)
target_link_libraries(go_to_pose_action_node ${PROJECT_NAME}_lib "${cpp_typesupport_target}" )

add_executable(test_position2d src/test_position2d.cpp)
target_link_libraries(test_position2d ${PROJECT_NAME}_lib "${cpp_typesupport_target}" )

install(TARGETS
   patrol_node
   direction_service_node
   direction_service_test
   patrol_with_service_node
   go_to_pose_action_node
   test_position2d
   DESTINATION lib/${PROJECT_NAME}
)

# ---------------------------------- COPYING ADDITIONAL FOLDERS ------------------------------------
install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
