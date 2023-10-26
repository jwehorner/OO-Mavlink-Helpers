##########################################
# Catch Installation
##########################################
Include(FetchContent)

set(FETCHCONTENT_QUIET FALSE)

FetchContent_Declare(
	Catch2
	GIT_REPOSITORY https://github.com/catchorg/Catch2.git
	GIT_TAG        v3.3.2
	GIT_PROGRESS TRUE
)

FetchContent_MakeAvailable(Catch2)

enable_testing()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")
set(CMAKE_C_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-arcs -ftest-coverage")

##########################################
# Google Test Targets
##########################################
add_executable(test_mavlink_helpers				"${CMAKE_SOURCE_DIR}/test/test_mavlink_helpers.cpp")
include_directories(test_mavlink_helpers		"${MAVLINK_HELPERS_INCLUDES_LIST}")
target_link_libraries(test_mavlink_helpers 		Catch2::Catch2)
if(WIN32)
	target_link_libraries(test_mavlink_helpers	wsock32 ws2_32)
endif()

##########################################
# Regular Test Targets
##########################################
add_executable(test_component 					"test_component.cpp")
include_directories(test_component 				"${MAVLINK_HELPERS_INCLUDES_LIST}")
if(WIN32)
	target_link_libraries(test_component		 wsock32 ws2_32)
endif()