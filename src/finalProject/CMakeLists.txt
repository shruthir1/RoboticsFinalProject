cmake_minimum_required(VERSION 3.5)
project(finalProject)

# Default C standards
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

include_directories(include)

# ----- Executables -----

add_library(finalProject src/navigation.cpp) #unsure 

add_executable(navigation_node src/navRevisedAnusha.cpp)
ament_target_dependencies(navigation_node
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
)

add_executable(perception_node src/perception.cpp)
ament_target_dependencies(perception_node
  rclcpp
  sensor_msgs
  geometry_msgs
)

add_executable(locomotion_node src/locomotion.cpp)
ament_target_dependencies(locomotion_node
  rclcpp
  geometry_msgs
)
# unsure
ament_export_targets(finalProjectlib HAS_LIBRARY_TARGET) 

# ----- Installation -----

install(TARGETS
  navigation_node
  perception_node
  locomotion_node
  EXPORT finalProjectlib #unsure
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
