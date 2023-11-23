// Windows specific include order fix.
#ifdef _WIN32
#include <WinSock2.h>
#endif

// C++ Standard Library Headers
#include <atomic>
#include <csignal>
#include <iostream>
#include <map>
#include <string>
#include <vector>

// Mavlink Headers
#include <mavlink.h>

// CLI 11 Headers
#include <CLI/CLI.hpp>

// Mavlink Helpers Headers
#include "MavlinkHelper.hpp"

const static inline std::string TEST_COMPONENT_NAME = "Mavlink Helpers Test Component";
std::atomic_bool interrupt_received = false;

void mission_download_callback(std::reference_wrapper<std::map<uint16_t, mavlink_mission_item_int_t>> items) {
	logging::console::print("Mission received with " + std::to_string(items.get().size()) + " mission items.", TEST_COMPONENT_NAME, logging::severity::info);
}

void current_sequence_callback(uint16_t seq) {
	logging::console::print("Current sequence set to mission item " + std::to_string(seq) + ".", TEST_COMPONENT_NAME, logging::severity::info);
}

void handle_signal(int signal) {
	interrupt_received.store(true);
}

int main(int argc, char* argv[]) {
	try {
		/**************************************************************************************************/
		/* Signal Configuration																			  */
		/**************************************************************************************************/
		signal(SIGABRT, handle_signal);  
		signal(SIGFPE, handle_signal);  
		signal(SIGILL, handle_signal);  
		signal(SIGINT, handle_signal);  
		signal(SIGSEGV, handle_signal);  
		signal(SIGTERM, handle_signal);  

		/**************************************************************************************************/
		/* Command Line Options																			  */
		/**************************************************************************************************/
		uint8_t system_id;
		uint8_t component_id;
		uint16_t receive_port;
		std::string ground_control_address;
		uint16_t ground_control_port;

		CLI::App app{TEST_COMPONENT_NAME};

		CLI::Option* system_id_option = app.add_option("-s, --system_id", system_id, "System ID of the test system.")
			->default_val(1);
		CLI::Option* component_id_option = app.add_option("-c, --component_id", component_id, "Component ID of the test system.")
			->default_val(7);
		CLI::Option* receive_port_option = app.add_option("-r, --receive_port", receive_port, "Port to receive Mavlink messages on.")
			->default_val(44444);
		CLI::Option* qgc_address_option = app.add_option("-g, --ground_control_address", ground_control_address, "Address of the ground control station to connect to.")
			->default_val("127.0.0.1")
			->check(CLI::ValidIPV4);
		CLI::Option* qgc_port_option = app.add_option("-p, --ground_control_port", ground_control_port, "Port of the ground control station to connect to.")
			->default_val(14550);

		CLI11_PARSE(app, argc, argv);

		logging::console::print(
			"Starting test component: \nSystem: " + std::to_string(system_id) +
			"\nComponent: " + std::to_string(component_id) +
			"\nReceiving port: " + std::to_string(receive_port) + 
			"\nGroundstation address: " + ground_control_address + ":" + std::to_string(ground_control_port) + ".",
			TEST_COMPONENT_NAME,
			logging::severity::info
		);

		/**************************************************************************************************/
		/* Mavlink Helpers Configuration																  */
		/**************************************************************************************************/
		MavlinkHelper mh = MavlinkHelper(system_id, component_id, receive_port, ground_control_address, ground_control_port);
		mh.heartbeat_helper.set_heartbeat_state(MAV_STATE_ACTIVE);
		mh.mission_helper.register_mission_downloaded_callback(MAV_MISSION_TYPE_MISSION, mission_download_callback);
		mh.mission_helper.register_mission_downloaded_callback(MAV_MISSION_TYPE_FENCE, mission_download_callback);
		mh.mission_helper.register_current_item_set_callback(current_sequence_callback);


		/**************************************************************************************************/
		/* Event Loop																					  */
		/**************************************************************************************************/
		while (!interrupt_received) {
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
		}
		logging::console::print("Interrupt received, exiting.", TEST_COMPONENT_NAME, logging::severity::info);
		mh.close();
	}
	catch (std::runtime_error e) {
		std::cout << e.what() << std::endl;
		return -1;
	}
    return 0;
}