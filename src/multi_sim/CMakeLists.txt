cmake_minimum_required(VERSION 3.8)
project(multi_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(tf2_ros REQUIRED)


add_executable(mission_setpoints src/exploration.cpp)
ament_target_dependencies(mission_setpoints rclcpp px4_msgs )

add_executable(arm_test src/arm_test.cpp)
ament_target_dependencies(arm_test rclcpp px4_msgs )

add_executable(arm_safe src/arm_safe.cpp)
ament_target_dependencies(arm_safe rclcpp px4_msgs )

add_executable(arm_all src/arm_all.cpp)
ament_target_dependencies(arm_all rclcpp px4_msgs )

add_executable(hover_test src/hover_test.cpp)
ament_target_dependencies(hover_test rclcpp px4_msgs )

add_executable(altitude_probe src/altitude_probe.cpp)
ament_target_dependencies(altitude_probe rclcpp px4_msgs )

add_executable(crossing_test src/crossing_test.cpp)
ament_target_dependencies(crossing_test rclcpp px4_msgs )

add_executable(stack_test src/stack_test.cpp)
ament_target_dependencies(stack_test rclcpp px4_msgs )

install(TARGETS
  mission_setpoints
  arm_test
  arm_safe
  arm_all
  hover_test
  altitude_probe
  crossing_test
  stack_test
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}
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
