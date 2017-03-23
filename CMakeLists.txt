cmake_minimum_required(VERSION 2.8.3)
project(proto_grasp_movement)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
	roscpp
	rospy
	roslib
	rviz_visual_tools
	tf
	tf_conversions
	cmake_modules
	controller_manager
	kdl_parser
	kdl_conversions
	qb_interface

)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES proto_grasp_movement
  CATKIN_DEPENDS roscpp rviz_visual_tools tf qb_interface
#  DEPENDS system_lib
	kdl_parser
)

# ============================================================ include 
include_directories(
	include
  	${catkin_INCLUDE_DIRS}
)


# ============================================================ EX: main 
add_executable(
	main 
	src/main.cpp
	src/pg_movement.cpp
)
add_dependencies(
	main 
	${${PROJECT_NAME}_EXPORTED_TARGETS} 
	${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
	main
  	${catkin_LIBRARIES}
)

# ============================================================ EX: hand republisher 
add_executable(
	hand_remap 
	src/hand_republisher.cpp
)
add_dependencies(
	hand_remap 
	${${PROJECT_NAME}_EXPORTED_TARGETS} 
	${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
	hand_remap
  	${catkin_LIBRARIES}
)