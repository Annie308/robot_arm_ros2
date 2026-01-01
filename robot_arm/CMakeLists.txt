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
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

#executables
add_executable(inverse_kin src/inverse_kin.cpp)
add_executable(start_motors src/motor_control.cpp)
add_executable(start_gyro src/gyro_sensor.cpp)
add_executable(set_target src/set_target.cpp)
add_executable(get_pos src/forward_kin.cpp)
add_executable(state_publisher src/state_publisher.cpp)

target_link_libraries(state_publisher PUBLIC
  ${robot_arm_interfaces_TARGETS}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  tf2_ros::tf2_ros
  ${tf2_geometry_msgs_TARGETS}
  rclcpp::rclcpp
)

target_include_directories(get_pos
  PUBLIC
    include                 # your package headers
    ${EIGEN3_INCLUDE_DIRS}  # Eigen headers
)

target_include_directories(inverse_kin
  PUBLIC
    include                 # your package headers
    ${EIGEN3_INCLUDE_DIRS}  # Eigen headers
)
target_link_libraries(inverse_kin PUBLIC
    ${robot_arm_interfaces_TARGETS}
    ${std_msgs_TARGETS}
    Eigen3::Eigen 
    rclcpp::rclcpp
  )
target_link_libraries(get_pos PUBLIC
    ${robot_arm_interfaces_TARGETS}
    ${std_msgs_TARGETS}
    Eigen3::Eigen
    rclcpp::rclcpp
  )
target_link_libraries(start_motors PUBLIC
    ${robot_arm_interfaces_TARGETS}
    ${std_msgs_TARGETS}
    rclcpp::rclcpp
    rclcpp_action::rclcpp_action
  )
target_link_libraries(start_gyro PUBLIC
    ${robot_arm_interfaces_TARGETS}
    ${std_msgs_TARGETS}
    rclcpp::rclcpp
  )
target_link_libraries(set_target PUBLIC
    ${robot_arm_interfaces_TARGETS}
    ${std_msgs_TARGETS}
    rclcpp::rclcpp
    rclcpp_action::rclcpp_action
  )

install(TARGETS 
  inverse_kin
  start_motors
  start_gyro
  set_target
  get_pos
  state_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()