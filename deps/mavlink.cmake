option(MAVLINK_GENERATED "Option for if the mavlink repo has dialect implementations in the generated/include directory or if it is in the root." ON)

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
