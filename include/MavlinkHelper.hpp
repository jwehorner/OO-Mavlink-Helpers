#ifndef MAVLINK_HELPER_HPP
#define MAVLINK_HELPER_HPP

// Microservice Helpers
#include "Heartbeat.hpp"
#include "Mission.hpp"

// Socket Library
#include <UDPSocket.hpp>

// Mavlink Library
#include <mavlink.h>

// System Libraries
#include <chrono>
#include <cstdint>
#include <iostream>
#include <map>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#define THREAD_EXIT_CHECK_INTERVAL_MS 1000

/**
 * @brief	Class MavlinkHelper provides helper functions for the Mavlink protocol and it's microservices.
 * @details	The class provides the ability to connect to a GCS by sending heartbeats, then download missions
 * 			as they become available. 
 */
class MavlinkHelper {
public:

	/**
	 * @brief 	Constructor for the Mavlink Helper class.
	 * @details	The constructor additionally launches the heartbeat and receive threads.
	 * @param 	system_id 				uint8_t system ID of the associated component
	 * @param 	component_id 			uint8_t component ID of the associated component
	 * @param 	local_port 				unsigned short port number that the component should communicate over.
	 * @param 	heartbeat_interval_ms 	unsigned int number of microseconds between each heartbeat (default 1000).
	 */
    MavlinkHelper(
		uint8_t system_id, uint8_t component_id, 
		unsigned short local_port,
		std::string remote_address = "127.0.0.1", unsigned short remote_port = 14550,
		unsigned int heartbeat_interval_ms = 1000
		) : 
			system_id(system_id), component_id(component_id),
			local_port(local_port),
			remote_address(remote_address), remote_port(remote_port),
			component_socket(local_port),
			mission_helper(system_id, component_id, &component_socket),
			heartbeat_helper(system_id, component_id, &component_socket, heartbeat_interval_ms)
	{
		/*****************************************
		* Member Initialization
		******************************************/
		// Flag for if the component should exit.
		close_component = false;
		
		/*****************************************
		* Socket Configuration
		******************************************/
		// Configure the address of the GCS that the component will be communicating with.
		component_socket.configure_remote_host(remote_port, remote_address);
		component_socket.set_socket_receive_timeout(THREAD_EXIT_CHECK_INTERVAL_MS);

		/*****************************************
		* Thread Initialization
		******************************************/
		// Launch the receive threads.
		std::thread receive_thread = std::thread(&MavlinkHelper::receive_thread, this);
		receive_thread.detach();
	}

	/**
	 * @brief Destructor for the MavlinkHelper class which sets the close flag then waits to exit.
	 */
	~MavlinkHelper() {
		heartbeat_helper.close();
		close();
	}

	void close() {
		close_component = true;
		std::this_thread::sleep_for(std::chrono::milliseconds(THREAD_EXIT_CHECK_INTERVAL_MS * 2));
	}


protected:
	/*****************************************
	* Networking Members
	******************************************/
	/// Port which the helper should communicate over.
    unsigned short local_port;
	/// Address to send Mavlink messages to.
	std::string remote_address;
	/// Port to send Mavlink messages to.
	unsigned short remote_port;
	/// UDP socket object that the helper and microservice helpers will use.
	UDPSocket component_socket;


	/*****************************************
	* Multithreading Members
	******************************************/
	/// Flag for if the child threads should close.
	bool close_component;


	/*****************************************
	* General Mavlink Members
	******************************************/
	/// System ID of the associated component.
    uint8_t system_id;
    /// Component ID of the associated component. 
    uint8_t component_id;

public:
	/*****************************************
	* Mission Microservice Helper
	******************************************/
	/// Public member so the mission data interface can be accessed.
	Mission mission_helper;
	/// Public member so the heartbeat interface can be accessed.
	Heartbeat heartbeat_helper;


protected:
	/*****************************************
	* Threads
	******************************************/
	
	/**
	 * @brief	Method receive_thread is a thread used to receive Mavlink messages and handle them accordingly.
	 */
	void receive_thread() {
		// Declare buffers to hold messages and statuses.
		mavlink_message_t msg;
		mavlink_status_t status;
		std::vector<char> buffer_vector = std::vector<char>();

		// While the component should continue operation,
		while(!close_component) {

			// Try to receive a message on the socket.
			try {
				buffer_vector = component_socket.receive();
			}
			// Catch and print any errors that occur.
			catch (std::runtime_error e) {
				std::cout << format_message("Error while receiving Mavlink message: \n" + std::string(e.what()));
				buffer_vector.clear();
			}

			if (!buffer_vector.empty()) {
				// When a message has been received, parse the bytes.
				for (char c : buffer_vector) {
					// If the bytes form a mavlink message,
					if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
						// Handle the specfic message ID,
						switch (msg.msgid)
						{					
						case MAVLINK_MSG_ID_MISSION_COUNT:
							mission_helper.handle_message_mission_count(msg);
							break;

						case MAVLINK_MSG_ID_MISSION_ITEM_INT:
							mission_helper.handle_message_mission_item_int(msg);
							break;

						case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
							mission_helper.handle_message_mission_set_current(msg);
							break;

						case MAVLINK_MSG_ID_MISSION_ITEM_REACHED:
							mission_helper.handle_message_mission_item_reached(msg);
							break;

						case MAVLINK_MSG_ID_HEARTBEAT:
							heartbeat_helper.handle_message_heartbeat(msg);
							break;

						default:
							// std::cout << format_message("Message ID " + std::to_string(msg.msgid) + " is unsupported by the client at this time.", "WARNING");
							break;
						}
					}
				}
				buffer_vector.clear();
			}
		}
	}

	/*****************************************
	* Utility Functions
	******************************************/
	std::string format_message(std::string message, std::string level = "ERROR") {		
		return "[Client Component] (" + level + ")\t\t\t" + message + "\n";
	}
};

#endif /* MAVLINK_HELPER_HPP */