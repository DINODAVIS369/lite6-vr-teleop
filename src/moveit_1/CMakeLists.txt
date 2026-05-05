cmake_minimum_required(VERSION 3.22)
project(moveit_1)

find_package(ament_cmake REQUIRED)

ament_package()

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})




install(
  FILES .setup_assistant
  DESTINATION share/${PROJECT_NAME}
)
