message(STATUS "Downloading logging-tools main.")

include(FetchContent)

set(FETCHCONTENT_QUIET FALSE)

FetchContent_Declare(
	logging_tools
  	GIT_REPOSITORY https://github.com/jwehorner/logging-tools.git
	GIT_TAG "main"
	GIT_PROGRESS TRUE
)

FetchContent_GetProperties(logging_tools)
if(NOT logging_tools_POPULATED)
  FetchContent_Populate(logging_tools)
endif()

message(STATUS "logging-tools main downloaded.")

set(logging_tools_ROOT_DIR "${FETCHCONTENT_BASE_DIR}/logging_tools-src" CACHE STRING "logging_tools Root Directory")
set(logging_tools_INCLUDE_DIR "${FETCHCONTENT_BASE_DIR}/logging_tools-src/include" "${FETCHCONTENT_BASE_DIR}/logging_tools-src/src" CACHE STRING "logging_tools Include File Location")