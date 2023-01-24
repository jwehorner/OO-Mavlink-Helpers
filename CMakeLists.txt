cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(MavlinkHelpers)

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

##########################
###  Global Variables  ###
##########################
set(MAVLINK_HELPERS_INCLUDES_LIST	"${CMAKE_SOURCE_DIR}/include" "${CMAKE_SOURCE_DIR}/deps" "${MAVLINK_INCLUDE_DIR}")

#################################
###  Compiler Specific Fixes  ###
#################################

########################
###  Subdirectories  ###
########################
add_subdirectory(test)
