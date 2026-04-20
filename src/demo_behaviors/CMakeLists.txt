cmake_minimum_required(VERSION 3.8)
project(demo_behaviors)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)



# Main executable

file(GLOB BEHAVIOR_SOURCES src/behaviors/*.cpp)

add_executable(run_bt
  src/run_bt.cpp
  ${BEHAVIOR_SOURCES}
)

ament_target_dependencies(run_bt
  rclcpp
  behaviortree_cpp
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  ament_index_cpp
)

# Specify header search path
target_include_directories(run_bt PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install executable
install(TARGETS run_bt
  DESTINATION lib/${PROJECT_NAME}
)


# Install behavior tree XML files
install(DIRECTORY behavior_trees/
  DESTINATION share/${PROJECT_NAME}/behavior_trees
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()