cmake_minimum_required(VERSION 3.4)
project(franka_position_servo)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  dynamic_reconfigure
  eigen_conversions
  franka_hw
  geometry_msgs
  hardware_interface
  tf
  tf_conversions
  message_generation
  pluginlib
  realtime_tools
  roscpp
  rospy
  franka_control
)

find_package(Eigen3 REQUIRED)
find_package(Franka 0.7.0 REQUIRED)

add_message_files(FILES
        JointCommand.msg
        RobotState.msg
        EndPointState.msg
        JointLimits.msg
        #JointControllerStates.msg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs actionlib_msgs franka_msgs)

generate_dynamic_reconfigure_options(
  cfg/joint_controller_params.cfg 
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES franka_position_servo
  CATKIN_DEPENDS
    controller_interface
    dynamic_reconfigure
    eigen_conversions
    franka_hw
    geometry_msgs
    control_msgs
    sensor_msgs
    actionlib_msgs
    franka_msgs
    hardware_interface
    tf
    tf_conversions
    message_runtime
    pluginlib
    realtime_tools
    roscpp
    franka_control
  DEPENDS Franka
)


add_library(franka_position_servo
#  src/cartesian_pose_example_controller.cpp
#  src/cartesian_velocity_example_controller.cpp
#  src/joint_position_example_controller.cpp
  src/joint_position_controller.cpp
#  src/joint_velocity_example_controller.cpp
)

add_dependencies(franka_position_servo
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  ${PROJECT_NAME}_gencpp
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(franka_position_servo PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_include_directories(franka_position_servo SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(franka_position_servo PUBLIC
  include
)

## Installation
install(TARGETS franka_position_servo
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES franka_position_servo_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
catkin_install_python(
  PROGRAMS scripts/move_to_start.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Tools
include(${CMAKE_CURRENT_LIST_DIR}/../cmake/ClangTools.cmake OPTIONAL
  RESULT_VARIABLE CLANG_TOOLS
)
if(CLANG_TOOLS)
  file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
  file(GLOB_RECURSE HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
  )
  add_format_target(franka_position_servo FILES ${SOURCES} ${HEADERS})
  add_tidy_target(franka_position_servo
    FILES ${SOURCES}
    DEPENDS franka_position_servo
  )
endif()
