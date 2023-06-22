cmake_minimum_required(VERSION 3.8)
project(e2f_hardware)

find_package(ament_cmake_auto REQUIRED)
find_package(dynamixel_workbench_toolbox REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(
  ${PROJECT_NAME}
  SHARED
  src/e2f.cpp
  src/hardware_interface.cpp
)

pluginlib_export_plugin_description_file(
  hardware_interface hardware_interface_plugin.xml
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE)
