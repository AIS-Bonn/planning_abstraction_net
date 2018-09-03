cmake_minimum_required(VERSION 2.8)

project(planning_abstraction_net)

find_package(catkin REQUIRED COMPONENTS 
	roscpp
	rospy
	message_generation 
	std_msgs 
	eigen_conversions
	cv_bridge
)
	
add_message_files(FILES
)

add_service_files(FILES
	GetPathCosts.srv
	GetBatchAbstraction.srv
	SetMapCrops.srv
)

generate_messages(DEPENDENCIES
	std_msgs
)

catkin_package(
	INCLUDE_DIRS include
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

add_executable(planner
	src/planner.cpp
	src/heuristic_server.cpp
	src/priority_queue_discr_pose.cpp
)
target_link_libraries(planner
	${catkin_LIBRARIES}
)
add_dependencies(planner
	${PROJECT_NAME}_generate_messages_cpp
)


add_executable(training_data_generator
	src/data_serializer.cpp
	src/height_map_generator_abstr_learning.cpp
	src/training_data_generator.cpp
)
target_link_libraries(training_data_generator
	${catkin_LIBRARIES}
)
add_dependencies(training_data_generator
	${PROJECT_NAME}_generate_messages_cpp
)
