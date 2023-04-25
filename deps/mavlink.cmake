include(FetchContent)

FetchContent_Declare(
	mavlink
	# Download the commit of the repository on 2021-10-22 to line up with the version used in MavNRC
	URL https://github.com/mavlink/c_library_v2/archive/537a0cd1be24bc72b8993b453122e74cef2702ff.zip
)

FetchContent_GetProperties(mavlink)
if(NOT mavlink_POPULATED)
	FetchContent_Populate(mavlink)
endif()

set(mavlink_INCLUDE_DIR "${FETCHCONTENT_BASE_DIR}/mavlink-src/common" CACHE STRING "Mavlink Include File Location")
