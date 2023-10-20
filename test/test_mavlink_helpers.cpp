// System Libraries
#include <iostream>
#include <memory>
#include <vector>

// Unit Test Headers
#include <catch2/catch_session.hpp>
#include <catch2/catch_test_macros.hpp>

// Mavlink Headers
#include <mavlink.h>

// Socket Headers
#include <udp_socket.hpp>

// Logging Headers
#include <LogConsole.hpp>

// Mavlink Helpers Headers
#include "MavlinkHelper.hpp"



/*************************************************************************************************
* Main Method
*************************************************************************************************/
int main( int argc, char* argv[] ) {
	logging::console::set_max_name_length(std::string("TestMavlinkHelpers").size());

  	int result = Catch::Session().run( argc, argv );

	return result;
}

/*************************************************************************************************
* Mavlink Helpers Members
*************************************************************************************************/
TEST_CASE("Check constructor.", "[MavlinkHelper]") {
	std::shared_ptr<MavlinkHelper> mh;
	REQUIRE_NOTHROW(mh = std::make_shared<MavlinkHelper>(251, 1, 44444));
	REQUIRE_NOTHROW(mh->close());
}

/*************************************************************************************************
* Heartbeat Members
*************************************************************************************************/
TEST_CASE("Check set_heartbeat_state.", "[Heartbeat]") {
	std::shared_ptr<MavlinkHelper> mh;
	REQUIRE_NOTHROW(mh = std::make_shared<MavlinkHelper>(251, 1, 44444));
	REQUIRE_NOTHROW(mh->heartbeat_helper.set_heartbeat_state(MAV_STATE_ACTIVE));

	logging::console::print("Waiting for QGC startup.", "TestMavlinkHelpers", logging::severity::info);

	while(mh->heartbeat_helper.get_components().empty()) {
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	
	REQUIRE(mh->heartbeat_helper.get_components().size() > 0);
	REQUIRE_NOTHROW(mh->close());
}

TEST_CASE("Check get_component_state.", "[Heartbeat]") {
	std::shared_ptr<MavlinkHelper> mh;
	REQUIRE_NOTHROW(mh = std::make_shared<MavlinkHelper>(251, 1, 44444));
	REQUIRE_NOTHROW(mh->heartbeat_helper.set_heartbeat_state(MAV_STATE_ACTIVE));

	logging::console::print("Waiting for QGC startup.", "TestMavlinkHelpers", logging::severity::info);

	while(mh->heartbeat_helper.get_components().empty()) {
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	std::vector<std::pair<uint8_t, uint8_t>> components;
	REQUIRE(mh->heartbeat_helper.get_components().size() > 0);
	REQUIRE_NOTHROW(components = mh->heartbeat_helper.get_components());
	for (auto c : components) {
		REQUIRE(c.first > 0);
		REQUIRE(c.second > 0);
		REQUIRE(mh->heartbeat_helper.get_component_state(c.first, c.second) > 0);
	}
	REQUIRE_NOTHROW(mh->close());
}

/*************************************************************************************************
* Mission Members
*************************************************************************************************/
TEST_CASE("Check has_downloaded_mission.", "[Mission]") {
	std::shared_ptr<MavlinkHelper> mh;
	REQUIRE_NOTHROW(mh = std::make_shared<MavlinkHelper>(251, 1, 44444));
	REQUIRE_NOTHROW(mh->heartbeat_helper.set_heartbeat_state(MAV_STATE_ACTIVE));

	logging::console::print("Waiting for QGC startup and mission upload.", "TestMavlinkHelpers", logging::severity::info);

	while(!mh->mission_helper.has_downloaded_mission(MAV_MISSION_TYPE_MISSION)) {
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}

	REQUIRE(mh->mission_helper.has_downloaded_mission(MAV_MISSION_TYPE_MISSION));

	REQUIRE_NOTHROW(mh->close());
}

TEST_CASE("Check get_downloaded_mission_item_int.", "[Mission]") {
	std::shared_ptr<MavlinkHelper> mh;
	REQUIRE_NOTHROW(mh = std::make_shared<MavlinkHelper>(251, 1, 44444));
	REQUIRE_NOTHROW(mh->heartbeat_helper.set_heartbeat_state(MAV_STATE_ACTIVE));

	logging::console::print("Waiting for QGC startup and mission upload.", "TestMavlinkHelpers", logging::severity::info);

	while(!mh->mission_helper.has_downloaded_mission(MAV_MISSION_TYPE_MISSION)) {
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}

	mavlink_mission_item_int_t mi;
	REQUIRE_NOTHROW(mi = mh->mission_helper.get_downloaded_mission_item_int(MAV_MISSION_TYPE_MISSION, 2));
	REQUIRE(mi.seq == 2);

	REQUIRE_NOTHROW(mh->close());
}

TEST_CASE("Check get_download_status.", "[Mission]") {
	std::shared_ptr<MavlinkHelper> mh;
	REQUIRE_NOTHROW(mh = std::make_shared<MavlinkHelper>(251, 1, 44444));
	REQUIRE_NOTHROW(mh->heartbeat_helper.set_heartbeat_state(MAV_STATE_ACTIVE));

	logging::console::print("Waiting for QGC startup and mission upload.", "TestMavlinkHelpers", logging::severity::info);

	std::pair<uint16_t, uint16_t> status;
	REQUIRE_NOTHROW(status = mh->mission_helper.get_download_status(MAV_MISSION_TYPE_MISSION));
	while(status.second == 0 || status.first < status.second - 1) {
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
		REQUIRE_NOTHROW(status = mh->mission_helper.get_download_status(MAV_MISSION_TYPE_MISSION));
	}

	REQUIRE(status.first == status.second - 1);

	REQUIRE_NOTHROW(mh->close());
}

TEST_CASE("Check register_mission_downloaded_callback.", "[Mission]") {
	std::shared_ptr<MavlinkHelper> mh;
	REQUIRE_NOTHROW(mh = std::make_shared<MavlinkHelper>(251, 1, 44444));
	REQUIRE_NOTHROW(mh->heartbeat_helper.set_heartbeat_state(MAV_STATE_ACTIVE));

	logging::console::print("Waiting for QGC startup and mission upload.", "TestMavlinkHelpers", logging::severity::info);

	std::atomic_bool downloaded_flag = false;

	REQUIRE_NOTHROW(mh->mission_helper.register_mission_downloaded_callback(
		MAV_MISSION_TYPE_MISSION,
		[&downloaded_flag](std::reference_wrapper<std::map<uint16_t, mavlink_mission_item_int_t>>) {
			downloaded_flag.store(true);
		}
	));

	while(!downloaded_flag.load()) {
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}

	REQUIRE(downloaded_flag.load());

	REQUIRE_NOTHROW(mh->close());
}