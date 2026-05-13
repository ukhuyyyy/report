cmake_minimum_required(VERSION 3.8)
project(diff_drive_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ========================
# Dependencies
# ========================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# 🔥 THÊM 3 DÒNG NÀY
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIAL REQUIRED libserial)
find_package(Eigen3 REQUIRED)
# ========================
# Executable
# ========================
add_executable(cmdvel_uart src/cmdvel_uart.cpp)

ament_target_dependencies(cmdvel_uart
  rclcpp
  geometry_msgs
  nav_msgs
  tf2                 # 🔥 THÊM
  tf2_ros             # 🔥 THÊM
  tf2_geometry_msgs   # 🔥 THÊM
)

target_link_libraries(cmdvel_uart Boost::system)

# ------------------------

add_executable(imu src/imu.cpp)

ament_target_dependencies(imu
  rclcpp
  geometry_msgs
  sensor_msgs
)
target_link_libraries(imu Boost::system)



add_library(interfaces SHARED src/interfaces.cpp)

target_include_directories(interfaces PUBLIC
  include
)
add_executable(noisy_controller src/noisy_controller.cpp)

ament_target_dependencies(noisy_controller
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)
install(TARGETS
  noisy_controller
  DESTINATION lib/${PROJECT_NAME}
)
target_include_directories(noisy_controller PUBLIC
  include
)

add_executable(simple_controller src/simple_controller.cpp)

ament_target_dependencies(simple_controller
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)
target_link_libraries(simple_controller
  Eigen3::Eigen
)
install(TARGETS
  simple_controller
  DESTINATION lib/${PROJECT_NAME}
)
target_include_directories(simple_controller PUBLIC
  include
)

ament_target_dependencies(interfaces
  rclcpp
  geometry_msgs
  sensor_msgs
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  LIBSERIAL
)
target_link_libraries(simple_controller
  Eigen3::Eigen
)
target_link_libraries(interfaces Boost::system)
#   
# Install executable
# ========================
install(TARGETS
  cmdvel_uart
  imu
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  interfaces
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

# ========================
# Install resource files
# ========================
install(
  DIRECTORY launch urdf config worlds map rviz
  DESTINATION share/${PROJECT_NAME}
)
add_executable(twist_relay src/twist_relay.cpp)
ament_target_dependencies(twist_relay rclcpp geometry_msgs)

install(TARGETS
twist_relay
DESTINATION lib/${PROJECT_NAME}
)


install(
  DIRECTORY include
  DESTINATION include
)




# ========================
# Testing (optional)
# ========================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
pluginlib_export_plugin_description_file(
  hardware_interface
  diff_drive_robot_interface.xml
)

install(FILES diff_drive_robot_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_libraries(interfaces)
ament_export_dependencies(hardware_interface pluginlib)

ament_package() 

