cmake_minimum_required(VERSION 3.8)
project(e2f_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

generate_parameter_library(
  gripper_command_controller_params src/gripper_command_controller_params.yaml
)

ament_auto_add_library(
  ${PROJECT_NAME} SHARED src/gripper_command_controller.cpp
)
target_link_libraries(${PROJECT_NAME} gripper_command_controller_params)

pluginlib_export_plugin_description_file(
  controller_interface controller_plugins.xml
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE)
