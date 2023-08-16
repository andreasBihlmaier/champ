cmake_minimum_required(VERSION 3.5)
project(champ_gazebo)

add_compile_options(-std=c++17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(champ REQUIRED)
find_package(champ_msgs REQUIRED)
find_package(ign_ros2_control REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ignition-cmake2 REQUIRED)
find_package(ignition-common4 REQUIRED)
find_package(ignition-utils1 REQUIRED)
find_package(ignition-math6 REQUIRED)
find_package(ignition-msgs8 REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-transport11 REQUIRED)

set(dependencies
  rclcpp
  urdf
  champ
  champ_msgs
  ign_ros2_control
  ros_gz_sim
)

install(PROGRAMS
  scripts/imu_sensor.py
  scripts/odometry_tf.py
  scripts/odometry.py
  DESTINATION bin
)

include_directories(
  include
  ${IGNITION-CMAKE_INCLUDE_DIRS}
  /usr/include/ignition/cmake2
  ${IGNITION-COMMON_INCLUDE_DIRS}
  ${IGNITION-UTILS_INCLUDE_DIRS}
  ${IGNITION-MATH_INCLUDE_DIRS}
  ${IGNITION-MSGS_INCLUDE_DIRS}
  ${IGNITION-GAZEBO_INCLUDE_DIRS}
  ${IGNITION-TRANSPORT_INCLUDE_DIRS}
  ${champ_INCLUDE_DIRS}
  ${champ_INCLUDE_DIRS}/champ/
  ${champ_INCLUDE_DIRS}/champ/champ/
)

add_executable(contact_sensor src/contact_sensor.cpp)
target_link_libraries(contact_sensor
  ${IGNITION-CMAKE_LIBRARIES}
  ${IGNITION-COMMON_LIBRARIES}
  ${IGNITION-UTILS_LIBRARIES}
  ${IGNITION-MATH_LIBRARIES}
  ${IGNITION-MSGS_LIBRARIES}
  ${IGNITION-GAZEBO_LIBRARIES}
  ${IGNITION-TRANSPORT_LIBRARIES}
)
ament_target_dependencies(contact_sensor ${dependencies})

install(TARGETS 
  contact_sensor
    DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()
