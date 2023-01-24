cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(MavlinkHelpers)

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

#############################
###  Git Submodule Setup  ###
#############################
find_package(Git)
if(GIT_FOUND AND EXISTS "${CMAKE_SOURCE_DIR}/.git")
# Update submodules as needed
    option(GIT_SUBMODULE "Check submodules during build" ON)
    if(GIT_SUBMODULE)
		message(STATUS "Submodule update")
        execute_process(COMMAND "${GIT_EXECUTABLE}" submodule update --init --recursive
                        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
                        RESULT_VARIABLE GIT_SUBMOD_RESULT)
		if(NOT GIT_SUBMOD_RESULT EQUAL "0")
            message(FATAL_ERROR "git submodule update --init --recursive failed with ${GIT_SUBMOD_RESULT}, please checkout submodules")
        endif()
    endif()
endif()

if(NOT EXISTS "${CMAKE_SOURCE_DIR}/deps/Socket/README.md")
    message(FATAL_ERROR "The submodules were not downloaded! GIT_SUBMODULE was turned off or failed. Please update submodules and try again.")
endif()

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
