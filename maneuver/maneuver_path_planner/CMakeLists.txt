cmake_minimum_required(VERSION 3.8)
project(maneuver_path_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap REQUIRED) # Если необходимо


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(octomap_publisher src/octomap_publisher.cpp)
target_link_libraries(octomap_publisher ${rclcpp_LIBRARIES} ${octomap_msgs_LIBRARIES} octomap) # Если необходимо
ament_target_dependencies(octomap_publisher rclcpp octomap_msgs)

install(TARGETS octomap_publisher
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

set(LIBRARIES
    # Components
    moveit_global_planner_component
    moveit_hybrid_planning_manager
    moveit_local_planner_component)
 


install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
 )
  
install(
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
