cmake_minimum_required(VERSION 3.8)
project(robot_arm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_arm_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp_action REQUIRED)

#executables
add_executable(get_angles src/inverse_kin.cpp)
add_executable(start_motors src/motor_control.cpp)
add_executable(start_gyro src/gyro_sensor.cpp)
add_executable(set_target src/set_target.cpp)
add_executable(get_pos src/forward_kin.cpp)

target_include_directories(get_angles
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    include_directories(${EIGEN3_INCLUDE_DIRS})
  )

target_include_directories(get_pos
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    include_directories(${EIGEN3_INCLUDE_DIRS})
  )

target_include_directories(start_motors
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_include_directories(start_gyro
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_include_directories(set_target
  PUBLIC
   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    )

ament_target_dependencies(get_angles rclcpp std_msgs robot_arm_interfaces rclcpp_action) 
ament_target_dependencies(get_pos rclcpp std_msgs robot_arm_interfaces)
ament_target_dependencies(start_motors rclcpp std_msgs robot_arm_interfaces rclcpp_action) 
ament_target_dependencies(start_gyro rclcpp std_msgs robot_arm_interfaces)
ament_target_dependencies(set_target rclcpp std_msgs robot_arm_interfaces rclcpp_action)

target_link_libraries(get_angles Eigen3::Eigen)
target_link_libraries(get_pos Eigen3::Eigen)

install(TARGETS 
  get_angles
  start_motors
  start_gyro
  set_target
  get_pos
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()