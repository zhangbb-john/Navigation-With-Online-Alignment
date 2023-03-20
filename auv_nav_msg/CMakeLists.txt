cmake_minimum_required(VERSION 3.0.2)
project(auv_nav_msg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
  tf2
  tf2_geometry_msgs
  tf2_ros
)

add_message_files(
  FILES
  DVL.msg
  USBLLONG.msg
  USBLANGLES.msg
  RECVIM.msg
  DVLAllInfo.msg
  State.msg
)



generate_messages(
  DEPENDENCIES
  sensor_msgs#   std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)


