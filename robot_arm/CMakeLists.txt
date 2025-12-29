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
add_executable(set_action src/user_input.cpp)

target_include_directories(get_angles
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

target_include_directories(set_action
  PUBLIC
   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    )

install(TARGETS 
  get_angles
  start_motors
  start_gyro
  set_action
  DESTINATION lib/${PROJECT_NAME}
)

ament_target_dependencies(get_angles rclcpp std_msgs robot_arm_interfaces rclcpp_action) 
ament_target_dependencies(start_motors rclcpp std_msgs robot_arm_interfaces rclcpp_action) 
ament_target_dependencies(start_gyro rclcpp std_msgs robot_arm_interfaces)
ament_target_dependencies(set_action rclcpp std_msgs robot_arm_interfaces rclcpp_action)

target_link_libraries(get_angles Eigen3::Eigen)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
