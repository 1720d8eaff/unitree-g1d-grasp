cmake_minimum_required(VERSION 3.8)

project(g1_d_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    meshes
    g1_d
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES
    g1_d_ros.urdf
    g1_d.urdf
    package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
