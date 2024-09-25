cmake_minimum_required(VERSION 3.8)
project(main_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(msg_files
  "msg/KeyState.msg"
  "msg/ServoAngle.msg"
  "msg/ArucoMarkers.msg"
  "msg/Robot2Control.msg"
)
set(srv_files
)
set(action_files
  "action/Navi.action"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  ${msg_files}
  ${action_files}
  DEPENDENCIES geometry_msgs std_msgs
)

ament_package()