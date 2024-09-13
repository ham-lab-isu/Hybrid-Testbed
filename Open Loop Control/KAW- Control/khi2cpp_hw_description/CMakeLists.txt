cmake_minimum_required(VERSION 3.5)
project(khi2cpp_hw_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY cx110l/urdf cx110l/rviz cx110l/meshes
  DESTINATION share/${PROJECT_NAME}/cx110l
)

install(
  DIRECTORY cx165l/urdf cx165l/rviz cx165l/meshes
  DESTINATION share/${PROJECT_NAME}/cx165l
)

ament_package()
