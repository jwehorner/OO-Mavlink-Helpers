#include "MavlinkHelper.hpp"

#include <mavlink.h>

#include <iostream>
#include <map>
#include <vector>

bool mission_received;
std::map<uint16_t, mavlink_mission_item_int_t> mission;

void mission_download_callback(std::reference_wrapper<std::map<uint16_t, mavlink_mission_item_int_t>> items) {
	mission_received = !items.get().empty();
	mission = items;
}

int main() {
	mission_received = false;
	try {
		MavlinkHelper mh = MavlinkHelper(251, 1, 44444);
		mh.heartbeat_helper.set_heartbeat_state(MAV_STATE_ACTIVE);
		mh.mission_helper.register_mission_downloaded_callback(MAV_MISSION_TYPE_MISSION, mission_download_callback);
		while (!mission_received) {
			Sleep(100);
		}
		std::cout << "Mission was received with " << std::to_string(mission.size()) << " items." << std::endl;
		mh.close();
	}
	catch (std::runtime_error e) {
		std::cout << e.what() << std::endl;
		return -1;
	}
    return 0;
}