cmake_minimum_required(VERSION 3.8)
project(mapping_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(my_custom_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rmw)
find_package(message_filters REQUIRED)

add_executable(mapping_sim_cpp src/mapping_sim_top_down_cpp.cpp)
ament_target_dependencies(mapping_sim_cpp rclcpp OpenCV sensor_msgs cv_bridge nav_msgs std_msgs rmw my_custom_msgs)

add_executable(mapping_real_cpp src/mapping_real_cpp.cpp)
ament_target_dependencies(mapping_real_cpp rclcpp OpenCV sensor_msgs cv_bridge nav_msgs std_msgs rmw message_filters my_custom_msgs)

install(TARGETS
  mapping_sim_cpp
  mapping_real_cpp
  DESTINATION lib/${PROJECT_NAME}
)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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
