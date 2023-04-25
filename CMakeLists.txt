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
add_compile_definitions(NOMINMAX)

##########################
###  Dependency Setup  ###
##########################
include(deps/mavlink.cmake)
include(deps/Socket.cmake)
include(deps/logging-tools.cmake)

##########################
###  Global Variables  ###
##########################
set(INCLUDES_LIST
	"${CMAKE_CURRENT_SOURCE_DIR}/include" 
	"${socket_INCLUDE_DIR}"
	"${mavlink_INCLUDE_DIR}"
	"${logging_tools_INCLUDE_DIR}"
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