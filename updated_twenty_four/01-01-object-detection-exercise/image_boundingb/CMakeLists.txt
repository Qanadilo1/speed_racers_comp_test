cmake_minimum_required(VERSION 2.8.3)
project(image_boundingb)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
roscpp 
rospy 
std_msgs 
genmsg 
message_generation)

add_message_files(
FILES 
bb.msg)

generate_messages(
DEPENDENCIES
std_msgs
)

## Declare a catkin package
catkin_package(
CATKIN_DEPENDS roscpp rospy message_runtime

)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})



