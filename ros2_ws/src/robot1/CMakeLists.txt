cmake_minimum_required(VERSION 3.8)
project(robot1)

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
find_package(geometry_msgs REQUIRED)
find_package(main_msgs REQUIRED)
find_package(htmd_manager_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  geometry_msgs
  main_msgs
  htmd_manager_msgs
  rclcpp_action
)

add_executable(robot1_core src/node/robot1_core.cpp)
ament_target_dependencies(robot1_core ${dependencies})
target_include_directories(robot1_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(robot1_navi src/node/robot1_navi.cpp)
ament_target_dependencies(robot1_navi ${dependencies})
target_include_directories(robot1_navi PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(robot1_nav1 src/node/robot1_nav1.cpp)
ament_target_dependencies(robot1_nav1 ${dependencies})
target_include_directories(robot1_nav1 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(robot1_auto src/node/robot1_auto.cpp)
ament_target_dependencies(robot1_auto ${dependencies})
target_include_directories(robot1_auto PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(robot1_key src/node/robot1_key.cpp)
ament_target_dependencies(robot1_key ${dependencies})
target_include_directories(robot1_key PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS
  robot1_core
  robot1_navi
  robot1_nav1
  robot1_auto
  robot1_key
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
