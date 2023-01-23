#ifndef MAVLINK_HELPER_HEARTBEAT_HPP
#define MAVLINK_HELPER_HEARTBEAT_HPP

// Socket Library
#include <Socket/UDPSocket.hpp>

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


/**
 * @brief	Class Heartbeat provides helper functions for the Mavlink protocol and it's microservices.
 * @details	The class provides the ability to connect to a GCS by sending heartbeats, then download missions
 * 			as they become available. 
 */
class Heartbeat {
public:

	friend class MavlinkHelper;

	/**
	 * @brief 	Constructor for the Mavlink Helper class.
	 * @details	The constructor additionally launches the heartbeat and receive threads.
	 * @param 	system_id 				uint8_t system ID of the associated component
	 * @param 	component_id 			uint8_t component ID of the associated component
	 * @param 	local_port 				unsigned short port number that the component should communicate over.
	 * @param 	heartbeat_interval_ms 	unsigned int number of microseconds between each heartbeat (default 1000).
	 */
    Heartbeat(
		uint8_t system_id, uint8_t component_id, 
		UDPSocket *component_socket, unsigned int heartbeat_interval_ms) : 
			system_id(system_id), component_id(component_id),
			component_socket(component_socket), heartbeat_interval_ms(heartbeat_interval_ms)
	{
		/*****************************************
		* Member Initialization
		******************************************/
		// Flag for if the component should exit.
		close_component = false;
		component_health = MAV_STATE_BOOT;
		
		/*****************************************
		* Thread Initialization
		******************************************/
		// Launch the heartbeat and receive threads.
		std::thread heartbeat_thread = std::thread(&Heartbeat::heatbeat_thread, this);
		heartbeat_thread.detach();
	}

	// Delete the copy constructors for the Heartbeat class as it could cause incorrect state.
	Heartbeat(const Heartbeat&) = delete;
    Heartbeat& operator=(Heartbeat const&) = delete;

	/**
	 * @brief Destructor for the Heartbeat class which sets the close flag then waits to exit.
	 */
	~Heartbeat() {
		close_component = true;
		std::this_thread::sleep_for(std::chrono::milliseconds(heartbeat_interval_ms * 2));
	}

	/**
	 * @brief 	Method set_heartbeat_state sets the state of the component that will be sent in the heartbeats.
	 * @param 	state	MAV_STATE representing the health of the component.
	 */
	void set_heartbeat_state(MAV_STATE state) {
		// Lock the state lock then set the member.
		std::unique_lock<std::mutex> state_lock(state_mutex);
		component_health = state;
	}

protected:
	/*****************************************
	* General Members
	******************************************/
	/// UDP socket object that the helper and microservice helpers will use.
	UDPSocket *component_socket;


	/*****************************************
	* Multithreading Members
	******************************************/
	/// Flag for if the child threads should close.
	bool close_component;
	/// Mutex to control access to the heartbeat state from different threads.
	std::mutex state_mutex;


	/*****************************************
	* General Mavlink Members
	******************************************/
	/// System ID of the associated component.
    uint8_t system_id;
    /// Component ID of the associated component. 
    uint8_t component_id;


	/*****************************************
	* Connection Microservice Members
	******************************************/
	/// Number of milliseconds between each heartbeat.
	unsigned int heartbeat_interval_ms;
	/// State of the component that will be sent in the heartbeats.
	MAV_STATE component_health;


	/*****************************************
	* Threads
	******************************************/
	
	/**
	 * @brief	Method heatbeat_thread is used to periodically send heartbeat messages with the status of the component.
	 */
	void heatbeat_thread() {
		// Create the buffers to store the HEARTBEAT message
		std::vector<char> buffer_vector = std::vector<char>();
		mavlink_message_t msg;
		uint8_t *buffer_uint8;
		int length;

		// While the component is still operating,
		while (!close_component) {
			// Lock the state lock before accessing the member.
			std::unique_lock<std::mutex> state_lock(state_mutex);
			// Pack a heartbeat message with the health of the component.
			length = mavlink_msg_heartbeat_pack(system_id, component_id, &msg, MAV_TYPE_HELICOPTER, MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0, component_health);
			state_lock.unlock();

			buffer_uint8 = (uint8_t *)malloc(length * 2);
			length = mavlink_msg_to_send_buffer(buffer_uint8, &msg);
			
			// Copy the byte array to the vector to send to the socket.
			buffer_vector.insert(buffer_vector.end(), &buffer_uint8[0], &buffer_uint8[length]);
			
			// Try sending the vector using the socket.
			try {
				component_socket->send(buffer_vector);
			}
			// Catch and print any errors that occur.
			catch (std::runtime_error e) {
				std::cout << format_message("Error while sending heartbeat: \n" + std::string(e.what()));
			}

			// Clear the vector and free the buffer. 
			buffer_vector.clear();
			free(buffer_uint8);

			// Sleep for the heartbeat interval.
			std::this_thread::sleep_for(std::chrono::milliseconds(heartbeat_interval_ms));
		}
	}


	/*****************************************
	* Utility Functions
	******************************************/
	std::string format_message(std::string message, std::string level = "ERROR") {		
		return "[Mavlink Heartbeat Helper] (" + level + ")\t\t\t" + message + "\n";
	}
};

#endif /* MAVLINK_HELPER_HEARTBEAT_HPP */