cmake_minimum_required(VERSION 3.8)
project(auto_car)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)


include_directories(include)
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_library(robot_library src/gpio.cpp)
target_include_directories(robot_library PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
# ament_target_dependencies(robot_library tf2 tf2_ros  cv_bridge rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs)


add_executable(demo_1 src/demo_1.cpp)
add_executable(camera src/camera.cpp)

ament_target_dependencies(demo_1 rclcpp std_msgs)
ament_target_dependencies(camera rclcpp std_msgs cv_bridge sensor_msgs )

target_link_libraries(demo_1 robot_library)


install(TARGETS
  demo_1
  camera
  DESTINATION lib/${PROJECT_NAME}
)


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

ament_package()
