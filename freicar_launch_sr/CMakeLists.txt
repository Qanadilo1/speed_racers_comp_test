cmake_minimum_required(VERSION 3.1)
project(freicar_launch_sr)

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")
set(CMAKE_CXX_FLAGS "-O3 -Wall -Wextra")

# C++14 used for make_unique. if not available, use C++11 & 
# reimplement make_unique in a header file or use default syntax

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
  message_generation
)

catkin_package(
	CATKIN_DEPENDS 
	message_runtime
	geometry_msgs
	sensor_msgs
	std_msgs
	roscpp
	INCLUDE_DIRS  # exporting shared headers
)
# installing shared headers
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
