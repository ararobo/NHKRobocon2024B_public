cmake_minimum_required(VERSION 3.8)
project(image)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(OCV_VERSION "4.5")
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(main_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV ${OCV_VERSION} REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(realsense2 REQUIRED)
find_package(realsense2_camera_msgs REQUIRED)

if(OpenCV_FOUND)

  set(dependencies
    rclcpp
    std_msgs
    geometry_msgs
    main_msgs
    sensor_msgs
    OpenCV
    cv_bridge
    realsense2_camera_msgs
    realsense2
  )


  add_executable(aruco_detector src/node/aruco_detector.cpp)
  ament_target_dependencies(aruco_detector ${dependencies} realsense2)
  target_include_directories(aruco_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include> 
  ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(aruco_detector ${OpenCV_LIBS} ${realsense2_LIBRARY})

  add_executable(ball_detector src/node/ball_detector.cpp)
  ament_target_dependencies(ball_detector ${dependencies})
  target_include_directories(ball_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  /usr/local/include/opencv4)
  target_link_libraries(ball_detector ${OpenCV_LIBS})

  add_executable(box_detector src/node/box_detector.cpp)
  ament_target_dependencies(box_detector ${dependencies})
  target_include_directories(box_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(box_detector ${OpenCV_LIBS})

  install(TARGETS
    aruco_detector
    ball_detector
    box_detector
    DESTINATION lib/${PROJECT_NAME})

endif()

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
