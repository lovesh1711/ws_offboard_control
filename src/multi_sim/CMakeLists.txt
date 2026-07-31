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

add_executable(hover_test src/hover_test.cpp)
ament_target_dependencies(hover_test rclcpp px4_msgs )

install(TARGETS
  mission_setpoints
  arm_test
  arm_safe
  hover_test
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
