cmake_minimum_required(VERSION 3.8)
project(khi2cpp_hw)

set(CMAKE_CXX_STANDARD 14)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(HW_IF_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  hardware_interface
)
set(REF_GEN_INCLUDE_DEPENDS
  kdl_parser
  rclcpp
  trajectory_msgs
)
set(CONTROLLER_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  controller_interface
  realtime_tools
  trajectory_msgs
)

# find dependencies
find_package(controller_manager)
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${HW_IF_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${REF_GEN_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${CONTROLLER_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(
  include
  ${ament_INCLUDE_DIRS}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# add an executable, target the dependencies
add_executable(main src/main.cpp)
ament_target_dependencies(main PUBLIC ${REF_GEN_INCLUDE_DEPENDS})

# add a library, target the dependencies
add_library(
  khi2cpp_hw
  SHARED
  src/khi_hw_interface.cpp
  src/khi_cnt_interface.cpp
)
ament_target_dependencies(khi2cpp_hw PUBLIC ${HW_IF_INCLUDE_DEPENDS} ${CONTROLLER_INCLUDE_DEPENDS} ${REF_GEN_INCLUDE_DEPENDS})

target_compile_features(khi2cpp_hw PUBLIC cxx_std_17)
target_include_directories(khi2cpp_hw PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/include>
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/include>
$<INSTALL_INTERFACE:include/khi2cpp_hw>
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface khi2cpp_hw.xml)
# Export controller plugins
pluginlib_export_plugin_description_file(controller_interface khi2cpp_hw.xml)

# INSTALL
install(
  DIRECTORY src/include/
  DESTINATION include/khi2cpp_hw
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/khi2cpp_hw
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/khi2cpp_hw
)
install(
    TARGETS main
    RUNTIME DESTINATION lib/khi2cpp_hw
)

# Install the khi2cpp_hw library
install(TARGETS khi2cpp_hw
  EXPORT export_khi2cpp_hw
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## EXPORTS
ament_export_targets(export_khi2cpp_hw HAS_LIBRARY_TARGET)
ament_export_dependencies(${HW_IF_INCLUDE_DEPENDS} ${REF_GEN_INCLUDE_DEPENDS} ${CONTROLLER_INCLUDE_DEPENDS})

ament_package()
