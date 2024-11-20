cmake_minimum_required(VERSION 3.8)
project(cpp_imu_sub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

## ADDED -------------------------

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Link pigpio library
find_library(PIGPIO_LIB pigpio REQUIRED)
find_library(RT_LIB rt REQUIRED)

add_executable(listener_rate src/imu_subscriber_rate.cpp)
ament_target_dependencies(listener_rate rclcpp sensor_msgs)

add_executable(listener_az src/imu_subscriber_az_echo.cpp)
ament_target_dependencies(listener_az rclcpp sensor_msgs)

add_executable(listener_az_timed src/imu_subscriber_az_echo_timed.cpp)
ament_target_dependencies(listener_az_timed rclcpp sensor_msgs)

add_executable(listener_az_led src/imu_subscriber_LED.cpp)
target_link_libraries(listener_az_led pigpiod_if2)
ament_target_dependencies(listener_az_led rclcpp sensor_msgs)

add_executable(listener_az_led2 src/imu_subscriber_LED_pigpio.cpp)
target_link_libraries(listener_az_led2 ${PIGPIO_LIB} ${RT_LIB})
ament_target_dependencies(listener_az_led2 rclcpp sensor_msgs)

install(TARGETS
  listener_rate
  listener_az
  listener_az_timed
  listener_az_led
  listener_az_led2
  DESTINATION lib/${PROJECT_NAME})

## END ADDED ---------------------

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
