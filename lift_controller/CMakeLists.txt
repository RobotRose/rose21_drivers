cmake_minimum_required(VERSION 2.8.3)
project(rose21_lift_controller)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rose_transformations
	rose_base_msgs
	std_msgs
	contact_sensor_msgs
	roscomm
	shared_variables
	rose_hardware_controller
	rose_hardware_comm
	)

catkin_package(
 INCLUDE_DIRS 
 	include
 LIBRARIES 

 CATKIN_DEPENDS 
 	roscpp
	rose_transformations
 DEPENDS 

)

add_executable( lift_controller 	
					src/lift_controller_node.cpp
					src/lift_controller.cpp
)
						

add_dependencies( lift_controller ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries( lift_controller ${catkin_LIBRARIES} )
