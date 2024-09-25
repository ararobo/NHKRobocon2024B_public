cmake_minimum_required(VERSION 3.8)
project(peripheral)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(main_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(htmd_manager_msgs REQUIRED)

set(dependencies
  rclcpp
  main_msgs
  std_msgs
  htmd_manager_msgs
  geometry_msgs
)

add_executable(servo_driver src/node/servo_driver.cpp src/lib/simple_udp.cpp src/config/can_data_configure.cpp)
ament_target_dependencies(servo_driver ${dependencies})
target_include_directories(servo_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(solenoid_driver src/node/solenoid_driver.cpp src/lib/simple_udp.cpp src/config/can_data_configure.cpp)
ament_target_dependencies(solenoid_driver ${dependencies})
target_include_directories(solenoid_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(robot2_driver src/node/robot2_driver.cpp src/lib/simple_udp.cpp)
ament_target_dependencies(robot2_driver ${dependencies})
target_include_directories(robot2_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(robot1_sensor src/node/robot1_sensor.cpp src/lib/simple_udp.cpp src/config/can_data_configure.cpp)
ament_target_dependencies(robot1_sensor ${dependencies})
target_include_directories(robot1_sensor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


install(TARGETS
  servo_driver
  solenoid_driver
  robot2_driver
  robot1_sensor
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

ament_package()
