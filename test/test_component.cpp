#include "MavlinkHelper.hpp"

#include <mavlink.h>

#include <iostream>
#include <map>
#include <vector>
#include <signal.h>

bool interrupt_received = false;

void mission_download_callback(std::reference_wrapper<std::map<uint16_t, mavlink_mission_item_int_t>> items) {
	std::cout << "Mission received with " << items.get().size() << std::endl;
}

void current_sequence_callback(uint16_t seq) {
	std::cout << "Current sequence set " << seq << std::endl;
}

void handle_signal(int signal) {
	interrupt_received = true;
}

int main() {
	try {
		// Register all the signal interrupts.
		signal(SIGABRT, handle_signal);  
		signal(SIGFPE, handle_signal);  
		signal(SIGILL, handle_signal);  
		signal(SIGINT, handle_signal);  
		signal(SIGSEGV, handle_signal);  
		signal(SIGTERM, handle_signal);  

		MavlinkHelper mh = MavlinkHelper(251, 1, 44444);
		mh.heartbeat_helper.set_heartbeat_state(MAV_STATE_ACTIVE);
		mh.mission_helper.register_mission_downloaded_callback(MAV_MISSION_TYPE_MISSION, mission_download_callback);
		mh.mission_helper.register_current_item_set_callback(current_sequence_callback);
		while (!interrupt_received) {
			Sleep(100);
		}
		std::cout << "Interrupt received, exiting." << std::endl;
		mh.close();
	}
	catch (std::runtime_error e) {
		std::cout << e.what() << std::endl;
		return -1;
	}
    return 0;
}