cmake_minimum_required(VERSION 3.8)
project(mecanum_hardware_interface)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE UDP INTERFACE
add_library(
mecanum_hardware_interface
  SHARED
  src/udp_hardware_interface.cpp
  src/micro_ros_hardware_interface.cpp
)

target_compile_features(mecanum_hardware_interface PUBLIC cxx_std_17)
target_include_directories(mecanum_hardware_interface PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/mecanum_hardware_interface>
)
target_link_libraries(mecanum_hardware_interface PUBLIC
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

ament_target_dependencies(mecanum_hardware_interface PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})


# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface mecanum_hardware_interface.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS 
    mecanum_hardware_interface
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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

ament_package()
