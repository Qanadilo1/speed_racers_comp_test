cmake_minimum_required(VERSION 2.8.3)
project(freicar_control_sr)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  dynamic_reconfigure
  geometry_msgs
  nav_msgs
  message_generation
  roscpp
  rospy
  std_msgs
  tf
  visualization_msgs
  tf2_ros
  tf2
  tf2_eigen
  tf2_geometry_msgs
  raiscar_msgs
  freicar_common
)

find_package(orocos_kdl REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
        CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs  tf2 tf2_ros rospy dynamic_reconfigure
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES controller
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${geos_INCLUDE_DIRS}
)

add_executable(pure_pursuit src/controller/pure_pursuit.cpp src/controller/controller.cpp )
add_executable(test_pps src/controller/test_pps.cpp src/controller/controller.cpp )
add_executable(test_path src/controller/test_path.cpp src/controller/controller.cpp )

target_link_libraries(pure_pursuit ${catkin_LIBRARIES}  ${geos_LIBRARIES})
target_link_libraries(test_pps ${catkin_LIBRARIES}  ${geos_LIBRARIES})
target_link_libraries(test_path ${catkin_LIBRARIES}  ${geos_LIBRARIES})
