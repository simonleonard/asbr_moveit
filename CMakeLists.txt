cmake_minimum_required(VERSION 3.22)
project(asbr_moveit)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config srdf rviz DESTINATION share/${PROJECT_NAME})
#install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
