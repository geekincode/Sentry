cmake_minimum_required(VERSION 3.8)
project(autoaim_debug)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(image_transport_plugins REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(autoaim_interfaces REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(debug_node src/debug_node.cpp)
target_include_directories(debug_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(debug_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  debug_node
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
  "image_transport"
  "image_transport_plugins"
  "camera_info_manager"
  "autoaim_interfaces"
  "tf2_geometry_msgs"
)

install(TARGETS debug_node
  DESTINATION lib/${PROJECT_NAME})

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
