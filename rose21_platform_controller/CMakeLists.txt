cmake_minimum_required(VERSION 2.8.3)
project(rose21_platform_controller)

find_package(catkin REQUIRED COMPONENTS
		operator_messaging
		roscpp
		rose_base_msgs
		rose_common
		rose_transformations
		rose_watchdog
		rose_hardware_controller
		server_multiple_client
		shared_variables
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
	rose_transformations
	rose_watchdog
	server_multiple_client
	shared_variables
 DEPENDS 

)

include_directories( include ${catkin_INCLUDE_DIRS} )

add_executable( rose21_platform_controller 	
					src/platform_controller_node.cpp
					src/platform_controller.cpp
)

add_dependencies( rose21_platform_controller ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries( rose21_platform_controller ${catkin_LIBRARIES} )
