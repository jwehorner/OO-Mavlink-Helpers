#include "MavlinkHelper.hpp"

#include <mavlink.h>

#include <iostream>
#include <vector>

int main() {
	try {
		MavlinkHelper c1 = MavlinkHelper(251, 1, 44444);
		c1.heartbeat_helper.set_heartbeat_state(MAV_STATE_ACTIVE);
		while (true) {
			Sleep(100);
			std::cout << 
				"Current item: " << std::to_string(c1.mission_helper.get_download_status(MAV_MISSION_TYPE_MISSION).first) << 
				",\t Total Count: " << c1.mission_helper.get_download_status(MAV_MISSION_TYPE_MISSION).second <<
			std::endl;
		}
	}
	catch (std::runtime_error e) {
		std::cout << e.what() << std::endl;
		return -1;
	}
    return 0;
}