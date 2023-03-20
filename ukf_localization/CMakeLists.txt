cmake_minimum_required(VERSION 3.0.2)
project(ukf_localization)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra")
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  message_runtime
  roscpp
  rospy
  std_msgs
  sensor_msgs
  turtlesim
  tf
  tf2
  tf2_geometry_msgs
  tf2_ros
  nav_msgs
  visualization_msgs
  auv_nav_msg
  rosgraph_msgs
  serial
)
find_package(  Ceres REQUIRED)




generate_messages(DEPENDENCIES std_msgs geometry_msgs)

include_directories(
    ./
    include/
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS} 
)
##localization
add_executable(localization_node
    src/localization.cpp
    src/localization_ros.cpp
    src/localization_node.cpp
    src/utils.cpp
    src/initial_usbl.cpp
)
add_dependencies(localization_node
    ${PROJECT_NAME}_generate_messages_cpp
    ${catkin_EXPORTED_TARGETS}
    ${EIGEN3_LIBRARIES}
)

target_link_libraries(localization_node
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
)
# pub_node
add_library(motion SHARED data_gen/motion.cpp)
add_executable(pub_node
    data_gen/pub_node.cpp
    src/utils.cpp)
target_link_libraries(pub_node
    motion
    ${catkin_LIBRARIES}
)
# traj_pub
add_executable(traj_pub_node
    visualization/traj_pub.cpp
    src/utils.cpp)
target_link_libraries(traj_pub_node
    ${catkin_LIBRARIES}
)   
# usbl
add_executable(usbl_node
    usbl/usbl_client_select.cpp)
target_link_libraries(usbl_node
    ${catkin_LIBRARIES}
)       


# add_executable(beacon_node
#     usbl/beacon/beacon_serial.cpp)
# target_link_libraries(beacon_node
#     ${catkin_LIBRARIES}
# )   

# add_executable(usbl_position_node
#     usbl/beacon/usbl_position.cpp)
# target_link_libraries(usbl_position_node
#     ${catkin_LIBRARIES}
# ) 
# add_executable(traj_usbl
#     usbl/beacon/traj_usbl.cpp)
# target_link_libraries(traj_usbl
#     ${catkin_LIBRARIES}
# )   

# gps 
add_executable(gps_fix
    gps/gps_fix.cpp)
target_link_libraries(gps_fix
    ${catkin_LIBRARIES}
)  
add_executable(traj_gps
    gps/traj_gps.cpp)
target_link_libraries(traj_gps
    ${catkin_LIBRARIES}
) 


 
