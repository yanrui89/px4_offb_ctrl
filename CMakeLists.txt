cmake_minimum_required(VERSION 3.0.2)
project(px4_offb_ctrl)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(Threads)
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  nav_msgs
  quadrotor_msgs
  roscpp
  std_msgs
  tf2
  tf2_ros
  eigen_conversions
)

add_subdirectory(offboard_controller_lib)
add_subdirectory(rapid_traj_gen)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  setInitGlobalPose.srv
  takeoffExternal.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES px4_offb_ctrl
 CATKIN_DEPENDS geometry_msgs mavros_msgs nav_msgs quadrotor_msgs roscpp std_msgs tf2 tf2_ros
#  DEPENDS system_lib
 DEPENDS Eigen3
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  libbspline/cpp/include
)

add_executable(${PROJECT_NAME}_node
  src/px4_offb_ctrl.cpp src/px4_offb_ctrl_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} Eigen3::Eigen offboard_controller_lib
)

add_executable(traj_gen_node
  src/traj_gen.cpp
)

target_link_libraries(traj_gen_node
  ${catkin_LIBRARIES} rapid_traj_gen_lib
)
