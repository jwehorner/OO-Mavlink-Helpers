#include <stdio.h>
#include <vector>

#include <gtest/gtest.h>

#include <mavlink.h>

#include <UDPSocket.hpp>

#include "MavlinkHelper.hpp"

using namespace std;

TEST(MavlinkHelperTest, Test_MavlinkHelper_constructor) {
	bool exception_thrown = false;
	try
	{
		MavlinkHelper mh = MavlinkHelper(251, 1, 44444);

		mh.close();
		exception_thrown = false;
	}
	catch (std::exception e)
	{
		exception_thrown = true;
	}
	ASSERT_FALSE(exception_thrown);
}

TEST(MavlinkHelperTest, Test_Heartbeat_set_heartbeat_state) {
	bool exception_thrown = false;
	try
	{
		MavlinkHelper mh = MavlinkHelper(251, 1, 44444);
		mh.heartbeat_helper.set_heartbeat_state(MAV_STATE_ACTIVE);

		while(mh.heartbeat_helper.get_components().empty()) {
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}

		ASSERT_GE(mh.heartbeat_helper.get_components().size(), 1);

		mh.close();
		exception_thrown = false;
	}
	catch (std::exception e)
	{
		exception_thrown = true;
	}
	ASSERT_FALSE(exception_thrown);
}

TEST(MavlinkHelperTest, Test_Mission_has_downloaded_mission) {
	bool exception_thrown = false;
	try
	{
		MavlinkHelper mh = MavlinkHelper(251, 1, 44444);
		mh.heartbeat_helper.set_heartbeat_state(MAV_STATE_ACTIVE);

		while(!mh.mission_helper.has_downloaded_mission(MAV_MISSION_TYPE_MISSION)) {
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}

		ASSERT_TRUE(mh.mission_helper.has_downloaded_mission(MAV_MISSION_TYPE_MISSION));

		mh.close();
		exception_thrown = false;
	}
	catch (std::exception e)
	{
		exception_thrown = true;
	}
	ASSERT_FALSE(exception_thrown);
}

TEST(MavlinkHelperTest, Test_Mission_get_downloaded_mission_item_int) {
	bool exception_thrown = false;
	try
	{
		MavlinkHelper mh = MavlinkHelper(251, 1, 44444);
		mh.heartbeat_helper.set_heartbeat_state(MAV_STATE_ACTIVE);

		while(!mh.mission_helper.has_downloaded_mission(MAV_MISSION_TYPE_MISSION)) {}

		mavlink_mission_item_int_t mi = mh.mission_helper.get_downloaded_mission_item_int(MAV_MISSION_TYPE_MISSION, 2);

		ASSERT_EQ(mi.seq, 2);

		mh.close();
		exception_thrown = false;
	}
	catch (std::exception e)
	{
		exception_thrown = true;
	}
	ASSERT_FALSE(exception_thrown);
}

TEST(MavlinkHelperTest, Test_Mission_get_download_status) {
	bool exception_thrown = false;
	try
	{
		MavlinkHelper mh = MavlinkHelper(251, 1, 44444);
		mh.heartbeat_helper.set_heartbeat_state(MAV_STATE_ACTIVE);

		std::pair<uint16_t, uint16_t> status = mh.mission_helper.get_download_status(MAV_MISSION_TYPE_MISSION);
		while(status.second == 0 || status.first < status.second - 1) {
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
			status = mh.mission_helper.get_download_status(MAV_MISSION_TYPE_MISSION);
		}

		ASSERT_EQ(status.first, status.second - 1);

		mh.close();
		exception_thrown = false;
	}
	catch (std::exception e)
	{
		exception_thrown = true;
	}
	ASSERT_FALSE(exception_thrown);
}


