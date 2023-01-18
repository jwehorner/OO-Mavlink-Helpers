cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(MavlinkComponentClient)

include(deps/mavlink.cmake)

############################
###  Configured Headers  ###
############################

#####################################
###  Global Compiler Definitions  ###
#####################################

##########################
###  Dependency Setup  ###
##########################

##########################
###  Global Variables  ###
##########################
set(includes_list	"${CMAKE_SOURCE_DIR}/include" "${CMAKE_SOURCE_DIR}/deps" "${MAVLINK_INCLUDE_DIR}")

#################################
###  Compiler Specific Fixes  ###
#################################

########################
###  Subdirectories  ###
########################
add_subdirectory(test)
