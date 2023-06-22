cmake_minimum_required(VERSION 3.8)
project(e2f_description)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_package(INSTALL_TO_SHARE meshes urdf)
