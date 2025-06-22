cmake_minimum_required(VERSION 3.8)
project(dynamixel_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(dynamixel_sdk REQUIRED)

# インクルードディレクトリの設定
include_directories(
  include
  ${dynamixel_sdk_INCLUDE_DIRS}
)

# メッセージファイル設定
set(msg_files
  "msg/DynamixelController.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
)

# 実行可能ターゲット名を変更して衝突を回避する
add_executable(dynamixel_controller_node
  src/dynamixel_controller.cpp
)

ament_target_dependencies(dynamixel_controller_node
  rclcpp
  std_msgs
  dynamixel_sdk
)

# 自動生成されたメッセージのためのリンク設定
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(dynamixel_controller_node ${cpp_typesupport_target})

install(TARGETS
  dynamixel_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
