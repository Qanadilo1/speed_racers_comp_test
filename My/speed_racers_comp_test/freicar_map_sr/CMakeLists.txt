cmake_minimum_required(VERSION 2.8.3)
cmake_policy(SET CMP0048 NEW)
project(freicar_map_sr VERSION "0.0.1" LANGUAGES CXX)

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")
set(CMAKE_CXX_FLAGS "-O0 -g -Wall -Wextra")
set(CMAKE_POSITION_INDEPENDENT_CODE ON) # needed for Thrift
set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_BUILD_TYPE Debug)

set(MAPLIB_DIR ${CMAKE_SOURCE_DIR}/MapCore)
set(MAPTHRIFT_DIR ${CMAKE_SOURCE_DIR}/MapThrift)
set(MAPTHRIFT_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/MapThrift)
set(MAPLIB_INCLUDE_DIRS ${MAPLIB_DIR}/include)

## finding packages
find_package(nanoflann)
find_package(Threads REQUIRED)
find_package(Eigen3 3 REQUIRED)
# used to find Boost ZLIB & Thrift but somehow they're no longer found/required

find_package(catkin REQUIRED COMPONENTS
	roscpp
	roslib
	visualization_msgs
	freicar_common
	raiscar_msgs
)
# creating the catkin package
catkin_package(
	CATKIN_DEPENDS roscpp
				   roslib
				   visualization_msgs
				   freicar_common
		           raiscar_msgs

	# DEPENDS thrift
	INCLUDE_DIRS include
				 MapThrift
				 MapCore/include
	LIBRARIES freicar_globalmap
			  freicar_map_framework
)
# include paths
include_directories(
	include
	${MAPLIB_INCLUDE_DIRS}
	${MAPTHRIFT_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
	${THRIFT_INCLUDE_DIRS}
	${Boost_INCLUDE_DIR}
	${EIGEN3_INCLUDE_DIR}
)
# freicar_globalmap lib ----------------------------------------------------------------------------
add_library(freicar_globalmap SHARED ${MAPLIB_DIR}/src/freicar_map.cpp
									 ${MAPLIB_DIR}/src/freicar_map_objects.cpp
									 ${MAPLIB_DIR}/src/freicar_map_helper.cpp
									 ${MAPLIB_DIR}/src/freicar_type_conversion.cpp
                             		 ${MAPTHRIFT_DIR}/gen-cpp/map_data_structure_constants.cpp
                             		 ${MAPTHRIFT_DIR}/gen-cpp/map_data_structure_types.cpp
)
# set_property(TARGET globalmap PROPERTY POSITION_INDEPENDENT_CODE ON)  # important ? probably not.
set_target_properties(
    freicar_globalmap PROPERTIES CXX_STANDARD 14  # allegedly important.
)
# linking relevant libraries
target_link_libraries(freicar_globalmap nanoflann::nanoflann)

# freicar_map_framework lib ------------------------------------------------------------------------
add_library(freicar_map_framework MapThrift/gen-cpp/MapComm.cpp
								  src/thrift_map_proxy.cpp
								  src/logic/map_info.cpp
								  src/logic/right_of_way.cpp
								  src/planning/lane_follower.cpp
								  src/planning/lane_star.cpp
)
target_link_libraries(freicar_map_framework
					  freicar_globalmap # created above
					  thrift
)
# freicar_map node ---------------------------------------------------------------------------
add_executable(${PROJECT_NAME}_node src/main.cpp)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${CMAKE_THREAD_LIBS_INIT}
										   ${catkin_LIBRARIES}
										   freicar_map_framework
										   pthread
										   Eigen3::Eigen
											yaml-cpp
)
# installing map library
install(TARGETS freicar_globalmap freicar_map_framework
		ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY MapThrift/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY MapCore/include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
