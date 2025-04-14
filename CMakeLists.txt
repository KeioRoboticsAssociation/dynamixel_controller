cmake_minimum_required(VERSION 3.8)
project(dynamixel_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ヘッダー用インクルードディレクトリを指定
include_directories(
  include
  ${dynamixel_sdk_INCLUDE_DIRS}  # dynamixel_sdk のインクルードディレクトリが設定されていれば利用
)

# 実行ファイルの作成
add_executable(dynamixel_controller
  src/dynamixel_controller.cpp
)

set(msg_files
  "msg/DynamixelController.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
)
# 依存関係の設定
ament_target_dependencies(dynamixel_controller
  rclcpp
  std_msgs
  dynamixel_sdk
)

# ※必要に応じて明示的にライブラリをリンクする場合は以下のように設定
# target_link_libraries(dynamixel_controller ${dynamixel_sdk_LIBRARIES})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 以下、ライセンス関連チェックをスキップする場合の設定例
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# install 設定（実行ファイルをインストールするパス）
install(TARGETS
  dynamixel_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
