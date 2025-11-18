cmake_minimum_required(VERSION 3.8)
project(mbot_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)

include_directories(include)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Pose2DArray.msg"
  DEPENDENCIES geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)

# Obstacle distance grid library
add_library(obstacle_distance_grid
  src/obstacle_distance_grid.cpp
)
ament_target_dependencies(obstacle_distance_grid
  nav_msgs
)

# Navigation node
add_executable(navigation_node
  src/navigation_node.cpp
  src/astar.cpp
)
ament_target_dependencies(navigation_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(navigation_node
  obstacle_distance_grid
  ${cpp_typesupport_target}
)

# A* test node
add_executable(astar_test
  src/astar_test.cpp
  src/astar.cpp
)
ament_target_dependencies(astar_test
  rclcpp
  nav_msgs
  geometry_msgs
)
target_link_libraries(astar_test
  obstacle_distance_grid
  ${cpp_typesupport_target}
)

# Exploration node
add_executable(exploration_node
  src/exploration_node.cpp
  src/astar.cpp
  src/frontier_explorer.cpp
)
ament_target_dependencies(exploration_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(exploration_node
  obstacle_distance_grid
  ${cpp_typesupport_target}
)

# Motion controller node
add_executable(motion_controller_diff
  src/motion_controller_diff.cpp
)
ament_target_dependencies(motion_controller_diff
  rclcpp
  nav_msgs
  geometry_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(motion_controller_diff
  ${cpp_typesupport_target}
)

# Install targets and resources
install(TARGETS navigation_node astar_test exploration_node motion_controller_diff
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS obstacle_distance_grid
  DESTINATION lib
)
install(DIRECTORY include maps launch rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()