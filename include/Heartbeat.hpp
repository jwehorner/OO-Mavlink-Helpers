#ifndef MAVLINK_HELPER_HEARTBEAT_HPP
#define MAVLINK_HELPER_HEARTBEAT_HPP

// Logging Tools Headers
#include <LogConsole.hpp>

// Socket Library
#include <udp_socket.hpp>

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
		std::shared_ptr<oo_socket::udp::socket> component_socket, unsigned int heartbeat_interval_ms) : 
			system_id(system_id), component_id(component_id),
			component_socket(component_socket), heartbeat_interval_ms(heartbeat_interval_ms)
	{
		/*****************************************
		* Member Initialization
		******************************************/
		// Flag for if the component should exit.
		close_component = false;
		finished_exiting = false;
		component_health = MAV_STATE_BOOT;
		component_status_map = std::map<std::pair<uint8_t, uint8_t>, uint8_t>();
		
		/*****************************************
		* Thread Initialization
		******************************************/
		// Launch the heartbeat and receive threads.
		std::thread heartbeat_thread = std::thread(&Heartbeat::heartbeat_thread, this);
		heartbeat_thread.detach();
	}

	// Delete the copy constructors for the Heartbeat class as it could cause incorrect state.
	Heartbeat(const Heartbeat&) = delete;
    Heartbeat& operator=(Heartbeat const&) = delete;

	/**
	 * @brief Destructor for the Heartbeat class which sets the close flag then waits to exit.
	 */
	~Heartbeat() {
		close();
	}

	void close() {
		if (!finished_exiting) {
			{
				// Gain access to the exit lock then set the exit flag.
				std::unique_lock<std::mutex> close_component_guard(close_component_mutex);
				close_component = true;
			}
			{
				// What for the finished exiting flag to be set by waiting on the exiting condition variable.
				std::unique_lock<std::mutex> finished_exiting_guard(finished_exiting_mutex);
				// While the condition variable is false, wait.
				while (!finished_exiting)
				{
					finished_exiting_condition_true.wait(finished_exiting_guard);
				}
				// Unlock the exiting mutex.
				finished_exiting_guard.unlock();
				logging::console::print("Heartbeat Helper successfully closed.", "Heartbeat", logging::severity::info);
			}
		}
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

	/**
	 * @brief 	Method get_component_state returns the latest state of a component.
	 * @warning	This method does not track when the last heartbeat was received and thus should not be used 
	 * 			for health monitoring in it's current state.
	 * @param 	system_id 		uint8_t system ID of the component to check the state of.
	 * @param 	component_id	uint8_t component ID of the component to check the state of.
	 * @return	uint8_t 		MAV_STATE state of the specified component, MAV_STATE_UNINIT if no
	 * 							heartbeat has been received yet.
	 */
	uint8_t get_component_state(uint8_t system_id, uint8_t component_id) {
		// Create a pair of system and component ID to check the state of
		std::pair<uint8_t, uint8_t> component = std::pair<uint8_t, uint8_t>(system_id, component_id);

		// Lock the component status map lock then set the member.
		std::unique_lock<std::mutex> component_status_map_lock(component_status_map_mutex);
		
		// If a heartbeat has been received for the component, return the latest state.
		if (component_status_map.find(component) != component_status_map.end()) {
			return component_status_map[component];
		}
		// Otherwise return un-initialised.
		else {
			return MAV_STATE_UNINIT;
		}
	}

	/**
	 * @brief 	Method get_components gets a list of the components for which heartbeats have been received
	 * @warning	This method does not track when the last heartbeat was received and thus should not be used 
	 * 			for health monitoring in it's current state.
	 * @return	std::vector<std::pair<uint8_t, uint8_t>> list of system and component IDs from which 
	 * 			heartbeats have been received.
	 */
	std::vector<std::pair<uint8_t, uint8_t>> get_components() {
		// Create an empty list of system and component IDs.
		std::vector<std::pair<uint8_t, uint8_t>> components = std::vector<std::pair<uint8_t, uint8_t>>();

		// Lock the component status map lock then set the member.
		std::scoped_lock<std::mutex> component_status_map_lock(component_status_map_mutex);

		// Add all the keys from the component status map.
		for (auto c : component_status_map) {
			components.push_back(c.first);
		}

		return components;
	}

protected:
	/*****************************************
	* General Members
	******************************************/
	/// UDP socket object that the helper and microservice helpers will use.
	std::shared_ptr<oo_socket::udp::socket> component_socket;


	/*****************************************
	* Multithreading Members
	******************************************/
	/// Flag for if the child threads should close.
	bool close_component;
	/// Mutex to protect the close flag.
    std::mutex close_component_mutex;
	/// Flag to indicate that the child thread has exited.
	bool finished_exiting;
	/// Mutex to protect access to the finished exiting flag.
	std::mutex finished_exiting_mutex;
	/// Condition variable that can be waited on for the child thread to exit.
	std::condition_variable finished_exiting_condition_true;
	/// Mutex to control access to the heartbeat state from different threads.
	std::mutex state_mutex;
	/// Mutex to control access to the system status map from different threads.
	std::mutex component_status_map_mutex;


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
	/// Map of system and component IDs to their latest status.
	std::map<std::pair<uint8_t, uint8_t>, uint8_t> component_status_map;


	/*****************************************
	* Threads
	******************************************/
	
	/**
	 * @brief	Method heartbeat_thread is used to periodically send heartbeat messages with the status of the component.
	 */
	void heartbeat_thread() {
		// Create the buffers to store the HEARTBEAT message
		std::vector<char> buffer_vector = std::vector<char>();
		mavlink_message_t msg;
		uint8_t *buffer_uint8;
		int length;
		bool continue_heartbeat = true;

		// While the component is still operating,
		while (continue_heartbeat) {
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
				logging::console::print("Error while sending heartbeat: \n" + std::string(e.what()), "Heartbeat", logging::severity::error);
			}

			// Clear the vector and free the buffer. 
			buffer_vector.clear();
			free(buffer_uint8);

			{
				// Gain access to the exit lock then check the exit flag.
				std::unique_lock<std::mutex> close_component_guard(close_component_mutex);
				continue_heartbeat = !close_component;
			}

			if (continue_heartbeat) {
				// Sleep for the heartbeat interval.
				std::this_thread::sleep_for(std::chrono::milliseconds(heartbeat_interval_ms));
			}
		}

		{
			// Lock access to the finished exiting flag then set it.
			std::unique_lock<std::mutex> finished_exiting_guard(finished_exiting_mutex);
			finished_exiting = true;
		}
		// Notify the finished exiting condition variable that the thread has exited.
		finished_exiting_condition_true.notify_all();
	}


	/*****************************************
	* Mavlink Message Handlers
	******************************************/
	
	/**
	 * @brief 	Method handle_message_heartbeat is used to handle a HEARTBEAT message.
	 * @details	This method is provided so it can be called as a handler every time a HEARTBEAT 
	 * 			message is received. Upon receiving a HEARTBEAT message the class will 
	 * 			store the current status of the component that sent the message. 
	 * @param 	msg	mavlink_message_t containing the HEARTBEAT message.
	 */
	void handle_message_heartbeat(mavlink_message_t msg) {
		// If the message ID is not a HEARTBEAT, print a warning.
		if (msg.msgid != MAVLINK_MSG_ID_HEARTBEAT) {
			logging::console::print("Message is not a HEARTBEAT message so it will be ignored: \n" + std::to_string(msg.msgid), "Heartbeat", logging::severity::warning);
			return;
		}

		// Store the state of the sending component
		component_status_map[std::pair<uint8_t, uint8_t>(msg.sysid, msg.compid)] = mavlink_msg_heartbeat_get_system_status(&msg);
	}
};

#endif /* MAVLINK_HELPER_HEARTBEAT_HPP */