cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(MavlinkHelpers)

#################
###  Options  ###
#################
option(BUILD_MAVLINK_HELPER_TESTS "Optionally download test dependancies and compile test cases." OFF)

############################
###  Configured Headers  ###
############################

#####################################
###  Global Compiler Definitions  ###
#####################################

##########################
###  Dependency Setup  ###
##########################
message(STATUS "Setting up dependencies.")
include(deps/mavlink.cmake)
add_subdirectory(deps/Socket)

##########################
###  Global Variables  ###
##########################
set(MAVLINK_HELPERS_INCLUDES_LIST
	"${CMAKE_CURRENT_SOURCE_DIR}/include" 
	"${SOCKET_INCLUDES_LIST}"
	"${MAVLINK_INCLUDE_DIR}"
	CACHE STRING "Mavlink Helpers Includes List"	
)

#################################
###  Compiler Specific Fixes  ###
#################################

########################
###  Subdirectories  ###
########################
if(BUILD_MAVLINK_HELPER_TESTS) 
	add_subdirectory(test)
endif()