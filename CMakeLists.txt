cmake_minimum_required(VERSION 3.22)
project(asbr_moveit)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules" ${CMAKE_MODULE_PATH})

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread serialization)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(moveit_common REQUIRED)
find_package(rviz2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(class_loader REQUIRED)

find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(FLANN REQUIRED)

include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
include_directories(SYSTEM ${EIGEN_INCLUDE_DIR})
include_directories( $(catkin_INCLUDE_DIRS) )

link_directories(${Boost_LIBRARY_DIRS})

add_library( assignment3_context SHARED src/assignment3_context.cpp)
ament_target_dependencies( assignment3_context rclcpp moveit_core moveit_msgs)
target_link_libraries( assignment3_context ${MOVEIT_LIB_NAME} ${Boost_LIBRARIES})

add_library( assignment3_plugin SHARED src/assignment3_plugin.cpp)
ament_target_dependencies( assignment3_plugin rclcpp class_loader moveit_core moveit_msgs pluginlib)
target_link_libraries(assignment3_plugin assignment3_context ${MOVEIT_LIB_NAME} ${Boost_LIBRARIES})

install(
  TARGETS assignment3_context assignment3_plugin
  EXPORT asbr_planners_assignment3Targets
  LIBRARY DESTINATION lib 
  ARCHIVE DESTINATION lib 
  RUNTIME DESTINATION bin 
)

pluginlib_export_plugin_description_file(moveit_core assignment3_plugin.xml)

ament_export_targets( asbr_planners_assignment3Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(moveit_core)

# Install the launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)

# Install the configuration files
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# Install any additional necessary files, such as setup assistant files
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
install(FILES assignment3_plugin.xml DESTINATION share/${PROJECT_NAME})

# Export package information
ament_package()
