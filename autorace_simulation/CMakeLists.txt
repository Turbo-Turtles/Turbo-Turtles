cmake_minimum_required(VERSION 3.8)
project(autorace_simulation)

find_package(ament_cmake REQUIRED COMPONENTS
  gazebo_ros
)

install(DIRECTORY launch models worlds
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()