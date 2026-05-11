cmake_minimum_required(VERSION 3.8)
project(ur3e_moveit_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(std_msgs REQUIRED)

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

add_executable(move_robot src/move_robot.cpp)
ament_target_dependencies(move_robot
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  moveit_msgs
  shape_msgs
  std_msgs
)

add_executable(test_pick_place src/test_pick_place.cpp)
ament_target_dependencies(test_pick_place
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  moveit_msgs
)
add_executable(test_accuracy src/test_accuracy.cpp)
ament_target_dependencies(test_accuracy
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  moveit_msgs
)

add_executable(pick_place_mtc src/pick_place_mtc.cpp)
ament_target_dependencies(pick_place_mtc
  rclcpp
  moveit_ros_planning_interface
  moveit_task_constructor_core
  geometry_msgs
  moveit_msgs
  shape_msgs
  std_msgs
  trajectory_msgs
)

install(TARGETS move_robot test_pick_place test_accuracy pick_place_mtc
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/camera_to_robot_goals.py
  scripts/eef_pose_publisher.py
  scripts/fake_finger_width_state.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
