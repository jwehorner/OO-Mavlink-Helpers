include(FetchContent)

FetchContent_Declare(
  mavlink
  URL https://github.com/mavlink/c_library_v2/archive/refs/heads/master.zip
)

FetchContent_GetProperties(mavlink)
if(NOT mavlink_POPULATED)
  FetchContent_Populate(mavlink)
endif()

set(MAVLINK_INCLUDE_DIR ${FETCHCONTENT_BASE_DIR}/mavlink-src/common CACHE STRING "Mavlink Include File Location")
