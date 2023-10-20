message(STATUS "Downloading Socket main.")

include(FetchContent)

set(FETCHCONTENT_QUIET FALSE)

FetchContent_Declare(
	socket
  	GIT_REPOSITORY https://github.com/jwehorner/OOSocket.git
	GIT_TAG "main"
	GIT_PROGRESS TRUE
)

FetchContent_GetProperties(socket)
if(NOT socket_POPULATED)
  FetchContent_Populate(socket)
endif()

message(STATUS "Socket main downloaded.")

set(socket_ROOT_DIR "${FETCHCONTENT_BASE_DIR}/socket-src" CACHE STRING "socket Root Directory")
set(socket_INCLUDE_DIR "${FETCHCONTENT_BASE_DIR}/socket-src/include" CACHE STRING "socket Include File Location")