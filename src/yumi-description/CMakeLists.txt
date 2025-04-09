cmake_minimum_required(VERSION 3.8)
project(yumi-description)

find_package(ament_cmake REQUIRED)

ament_environment_hooks("hooks/resource_paths.dsv.in")
install(DIRECTORY meshes urdf hooks DESTINATION share/${PROJECT_NAME})

ament_package()
