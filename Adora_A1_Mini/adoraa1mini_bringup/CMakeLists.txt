cmake_minimum_required(VERSION 3.8)
project(adoraa1mini_bringup)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


set(CMAKE_INSTALL_RPATH /usr/local/lib)
find_package(serial REQUIRED)
find_package(rclcpp QUIET)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

 
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

# 添加生成消息的依赖
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Control1.msg"
  "msg/Dt1.msg"
  "msg/Dt2.msg"
  "msg/Error.msg"
  DEPENDENCIES std_msgs  # 如果使用了 time/duration 等类型
)

 
ament_export_dependencies(rosidl_default_runtime) 

find_package(rosidl_typesupport_cpp REQUIRED)
include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp  # 自动生成头文件路径
  ${ament_INCLUDE_DIRS}  # 自动包含所有ament依赖路径
)




add_executable(adoraa1mini_bringup_node src/adoraa1mini_bringup_V2.cpp)

ament_target_dependencies(adoraa1mini_bringup_node 
rclcpp 
std_msgs
sensor_msgs
tf2
nav_msgs
tf2_geometry_msgs
serial
rosidl_typesupport_cpp
${PROJECT_NAME}  # 关键链接自身消息
)

install(TARGETS adoraa1mini_bringup_node
DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include
DESTINATION share/${PROJECT_NAME}
)

ament_package()
