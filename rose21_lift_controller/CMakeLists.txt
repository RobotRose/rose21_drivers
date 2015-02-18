cmake_minimum_required(VERSION 2.8.3)
project(rose21_lift_controller)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	contact_sensor_msgs
	roscomm
	roscpp
	rose_base_msgs
	rose_hardware_comm
	rose_hardware_controller
	shared_variables
	std_msgs
	)

catkin_package(
 INCLUDE_DIRS 
 	include
 LIBRARIES 

 CATKIN_DEPENDS 
	contact_sensor_msgs
	roscomm
	roscpp
	rose_base_msgs
	rose_hardware_comm
	rose_hardware_controller
	shared_variables
	std_msgs
 DEPENDS 

)

set(CMAKE_CXX_FLAGS "-std=c++0x -g") #! @todo OH [IMPR]: Remove -g flag in release?

include_directories( include ${catkin_INCLUDE_DIRS} )

add_executable( lift_controller 	
					src/lift_controller_node.cpp
					src/lift_controller.cpp
)
						

add_dependencies( lift_controller ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries( lift_controller ${catkin_LIBRARIES} )
