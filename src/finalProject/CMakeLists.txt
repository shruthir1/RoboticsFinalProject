cmake_minimum_required(VERSION 3.5)
project(finalProject)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


add_library(navigator src/navigation.cpp)
add_executable(locomotion_node src/locomotion.cpp)
add_executable(nav src/nav.cpp src/navigation.cpp)
add_executable(perception_node src/perception.cpp)

target_include_directories(navigator
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_include_directories(nav
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_include_directories(perception_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_include_directories(locomotion_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)



ament_target_dependencies(navigator rclcpp geometry_msgs lifecycle_msgs rclcpp_action nav2_msgs)
ament_export_targets(navigatorlib HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp geometry_msgs lifecycle_msgs rclcpp_action nav2_msgs)
ament_target_dependencies(nav rclcpp geometry_msgs lifecycle_msgs rclcpp_action nav2_msgs)
ament_target_dependencies(locomotion_node rclcpp geometry_msgs)
ament_target_dependencies(perception_node rclcpp sensor_msgs geometry_msgs)
# ament_target_dependencies(nav rclcpp geometry_msgs lifecycle_msgs rclcpp_action nav2_msgs)


install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS navigator
  EXPORT navigatorlib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
  )

install(TARGETS nav DESTINATION lib/${PROJECT_NAME})
install(TARGETS perception_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS locomotion_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})


ament_package()