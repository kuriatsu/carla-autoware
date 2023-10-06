cmake_minimum_required(VERSION 3.8)
project(carla_autoware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(carla_msgs REQUIRED)
find_package(autoware_auto_vehicle_msgs REQUIRED)
find_package(autoware_auto_control_msgs REQUIRED)
find_package(tier4_debug_msgs REQUIRED)
find_package(tier4_system_msgs REQUIRED)
find_package(autoware_adapi_v1_msgs REQUIRED)


add_executable(carla_vehicle_interface_node src/carla_vehicle_interface.cpp)
ament_target_dependencies(carla_vehicle_interface_node rclcpp carla_msgs autoware_auto_vehicle_msgs nav_msgs autoware_auto_control_msgs)
install(TARGETS carla_vehicle_interface_node
  DESTINATION lib/${PROJECT_NAME})

add_executable(pseudo_localization_node src/pseudo_localization.cpp)
ament_target_dependencies(pseudo_localization_node rclcpp carla_msgs nav_msgs tier4_debug_msgs tier4_system_msgs autoware_adapi_v1_msgs)
install(TARGETS pseudo_localization_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

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
