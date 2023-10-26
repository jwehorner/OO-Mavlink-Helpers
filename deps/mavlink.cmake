message(STATUS "Downloading Mavlink.")

option(MAVLINK_GENERATED "Option for if the mavlink repo has dialect implementations in the generated/include directory or if it is in the root." ON)

if(NOT DEFINED CACHE{MAVLINK_REPO})
	message(WARNING	"Mavlink repository not set, defaulting open-source pre-generated Mavlink.")
	set(MAVLINK_REPO 		"https://github.com/mavlink/c_library_v2.git")
	set(MAVLINK_GENERATED	OFF)
endif()
if(NOT DEFINED CACHE{MAVLINK_REPO_TAG})
	message(WARNING	"Mavlink repository tag not set, defaulting to master.")
	set(MAVLINK_REPO_TAG	"master")
endif()
if(NOT DEFINED CACHE{MAVLINK_DIALECT})
	message(WARNING	"Mavlink dialect not set, defaulting to common.")
	set(MAVLINK_DIALECT		"common")
endif()

include(FetchContent)

FetchContent_Declare(
	mavlink
	GIT_REPOSITORY 	"${MAVLINK_REPO}"
	GIT_TAG			"${MAVLINK_REPO_TAG}"
	GIT_PROGRESS	TRUE
)

FetchContent_GetProperties(mavlink)
if(NOT mavlink_POPULATED)
	FetchContent_Populate(mavlink)
endif()

if (MAVLINK_GENERATED)
	set(mavlink_INCLUDE_DIR "${FETCHCONTENT_BASE_DIR}/mavlink-src/generated/include/mavlink/v2.0/${MAVLINK_DIALECT}" CACHE STRING "Mavlink Include File Location")
else()
	set(mavlink_INCLUDE_DIR "${FETCHCONTENT_BASE_DIR}/mavlink-src/${MAVLINK_DIALECT}" CACHE STRING "Mavlink Include File Location")
endif()
message(STATUS "Downloaded Mavlink ${MAVLINK_REPO_TAG} (${MAVLINK_REPO}).")
