cmake_minimum_required(VERSION 2.8.3)
project(rose21_power_controller)

find_package(catkin REQUIRED COMPONENTS
		operator_messaging
		roscpp
		rose_base_msgs
		rose_common
		rose_hardware_controller
		rose_shared_variables
		std_msgs
	)


catkin_package(
 INCLUDE_DIRS 
 	include
 LIBRARIES 

 CATKIN_DEPENDS
	operator_messaging
	roscpp
	rose_base_msgs
	rose_common
	rose_hardware_controller
	rose_shared_variables
	std_msgs
 DEPENDS 

)

set(CMAKE_CXX_FLAGS "-std=c++0x -g") #! @todo OH [IMPR]: Remove -g flag in release?

include_directories( include ${catkin_INCLUDE_DIRS} )

add_executable( rose21_power_controller 	
					src/power_controller_node.cpp
					src/power_controller.cpp
)

add_dependencies( rose21_power_controller ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries( rose21_power_controller ${catkin_LIBRARIES} )
