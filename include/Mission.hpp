#ifndef MAVLINK_HELPER_MISSION_HPP
#define MAVLINK_HELPER_MISSION_HPP

// Socket Library
#include <UDPSocket.hpp>

// Mavlink Library
#include <mavlink.h>

// System Libraries
#include <cstdint>
#include <functional>
#include <iostream>
#include <map>
#include <mutex>
#include <string>
#include <vector>

/**
 * @brief 	Class Mission is a helper class for the mission microservice of Mavlink.
 * @details	The class provides handler methods that can be called as Mavlink messages are 
 * 			received. Upon receiving mission messages the class will attempt to download  
 * 			each mission that is advertised. Downloaded missions can then be accessed using
 * 			the provided public interface.
 */
class Mission {
public:
	friend class MavlinkHelper;

	/**
	 * @brief 	Constructor for the Mission microservice helper class
	 * @param 	system_id 			uint8_t system ID of the associated component
	 * @param 	component_id 		uint8_t component ID of the associated component
	 * @param 	component_socket 	UDPSocket object that the component is interfacing over
	 */
    Mission(
		uint8_t system_id, uint8_t component_id, 
		std::shared_ptr<UDPSocket> component_socket) : 
			system_id(system_id), component_id(component_id),
			component_socket(component_socket)
	{
		/*****************************************
		* Member Initialization
		******************************************/		
		// Initialise the maps for handling current downloads.
		mission_current_download_count_map = std::map<uint8_t, uint16_t>();
		mission_current_download_item_map = std::map<uint8_t, uint16_t>();
		current_sequence_number = 0;
		
		// Initialise the maps for storing downloaded missions and in-progress downloads.
		mission_downloaded = std::map<uint8_t, std::map<uint16_t, mavlink_mission_item_int_t>>();
		mission_in_progress_buffer = std::map<uint8_t, std::map<uint16_t, mavlink_mission_item_int_t>>();
	}

	// Delete the copy constructors for the Mission class as it could cause incorrect state.
	Mission(const Mission&) = delete;
    Mission& operator=(Mission const&) = delete;

	/**
	 * @brief 	Method get_downloaded_mission_item_int gets a mission item from a fully downloaded mission.
	 * @param 	mission_type 				MAV_MISSION_TYPE_MISSION type of the mission
	 * @param 	mission_item_number 		uint16_t sequence number of the item within the mission
	 * @return	mavlink_mission_item_int_t 	structure for the mission item
	 * @throws 	invalid_argument if the mission type or item has not been downloaded.
	 */
	mavlink_mission_item_int_t get_downloaded_mission_item_int(uint8_t mission_type, uint16_t mission_item_number) {
		// Lock access to the mission items before accessing them.
		std::unique_lock mission_items_lock(mission_items_mutex);

		// If a downloaded mission of the correct type exists,
		if (mission_downloaded.find(mission_type) != mission_downloaded.end()) {
			// And if the downloaded mission contains an item with the correct number.
			if (mission_downloaded[mission_type].find(mission_item_number) != mission_downloaded[mission_type].end()) {
				// Return the mission item.
				return mission_downloaded[mission_type][mission_item_number];
			}
			// Else if the mission does not contain the mission item number, throw an error.
			else {
				mission_items_lock.unlock();
				throw std::invalid_argument(
					format_message(
						"Mission type " + std::to_string(mission_type) + 
						" does not contain a mission item with number " + std::to_string(mission_item_number),
						"ERROR"
					)
				);
			}
		}
		// Else if the mission type has not been downloaded.
		else {
			mission_items_lock.unlock();
			throw std::invalid_argument(
				format_message("Mission type " + std::to_string(mission_type) + " does not have any missions downloaded.")
			);
		}
	}

	/**
	 * @brief 	Method get_download_status gets the status of a download.
	 * @param 	mission_type 					MAV_MISSION_TYPE_MISSION type of the mission
	 * @return	std::pair<uint16_t, uint16_t>	pair where the first element is the item that is currently being 
	 * 											downloaded (starting at 0) and the second element is the total 
	 * 											count of the mission. Returns <count - 1, count> if the mission 
	 * 											has already been downloaded. If the mission is not being downloaded
	 * 											and has not been downloaded the pair <0, 0> is returned.
	 */
	std::pair<uint16_t, uint16_t> get_download_status(uint8_t mission_type) {
		// Lock access to the mission items before accessing them.
		std::unique_lock mission_items_lock(mission_items_mutex);

		// If the mission is or has been being downloaded, and the status of the download exists,
		if ((mission_in_progress_buffer.find(mission_type) != mission_in_progress_buffer.end() || 
			mission_downloaded.find(mission_type) != mission_downloaded.end()) &&
			mission_current_download_item_map.find(mission_type) != mission_current_download_item_map.end() &&
			mission_current_download_count_map.find(mission_type) != mission_current_download_count_map.end())
		{
			// Return a pair where the first element is the current item and the second item is the count.
			return std::pair<uint16_t, uint16_t> (
				mission_current_download_item_map[mission_type],
				mission_current_download_count_map[mission_type]
			);
		}
		// Else if the mission has not been downloaded,
		else {
			// Return a pair where the item and count are both 0.
			return std::pair<uint16_t, uint16_t> (0, 0);
		}
	}

	/**
	 * @brief 	Method has_downloaded_mission checks whether a mission type has been downloaded.
	 * @param 	mission_type	MAV_MISSION_TYPE_MISSION type of the mission
	 * @return	bool			true if a mission of the correct type has been downloaded, false otherwise.  
	 */
	bool has_downloaded_mission(uint8_t mission_type) {
			// Lock access to the mission items before accessing them.
		std::unique_lock mission_items_lock(mission_items_mutex);

		return mission_downloaded.find(mission_type) != mission_downloaded.end();
	}

	/**
	 * @brief 	Method register_mission_downloaded_callback stores a callback function to be invoked when the 
	 * 			specific mission type has been downloaded. 
	 * @param 	mission_type	MAV_MISSION_TYPE_MISSION type of the mission
	 * @param 	callback 		std::function which accepts a map of item numbers to mission items as a parameter.
	 */
	void register_mission_downloaded_callback(uint8_t mission_type, 
		std::function<void(std::reference_wrapper<std::map<uint16_t, mavlink_mission_item_int_t>>)> callback)
	{
		// Lock access to the callbacks before accessing them.
		std::unique_lock mission_items_lock(mission_items_mutex);

		mission_downloaded_callback_map[mission_type] = callback;
	}

	/**
	 * @brief	Method register_current_item_set_callback stores a callback function to be invoked when the 
	 * 			current sequence number of the mission has been set.
	 * @param 	callback 	std::function which accepts a uint16_t sequence number as a parameter.
	 */
	void register_current_item_set_callback(std::function<void(uint16_t)> callback)
	{
		// Lock access to the callbacks before accessing them.
		std::unique_lock mission_items_lock(mission_items_mutex);
		current_item_set_callback_vector.push_back(callback);
	}

	

protected:
	/*****************************************
	* General Members
	******************************************/
	/// UDP Socket that the component using the mission helper is communicating over.
	std::shared_ptr<UDPSocket> component_socket;

	/*****************************************
	* Multithreading Members
	******************************************/
	/// Mutex to control access to the mission items from different threads.
	std::mutex mission_items_mutex;

	/*****************************************
	* General Mavlink Members
	******************************************/
	/// System ID of the associated component.
    uint8_t system_id;
	/// Component ID of the associated component.
    uint8_t component_id;


	/*****************************************
	* Mission Microservice Members
	******************************************/
	/// Map storing the total number of mission items for a mission type.
	std::map<uint8_t, uint16_t> mission_current_download_count_map;
	/// Map storing the current mission item being downloaded for a mission type.
	std::map<uint8_t, uint16_t> mission_current_download_item_map;

	/// Map storing each type of mission that has been downloaded alongside it's items.
	std::map<uint8_t, std::map<uint16_t, mavlink_mission_item_int_t>> mission_downloaded;
	/// Map storing each type of mission that is currently being downloaded alongside it's items.
	std::map<uint8_t, std::map<uint16_t, mavlink_mission_item_int_t>> mission_in_progress_buffer;
	/// Current sequence number of the mission.
	uint16_t current_sequence_number;
	/// Map to store callback function to be called when a specific mission type has been downloaded.
	std::map<uint8_t, std::function<void(std::reference_wrapper<std::map<uint16_t, mavlink_mission_item_int_t>>)>> mission_downloaded_callback_map;
	/// Vector to store callback functions to be called when the current sequence number has been set.
	std::vector<std::function<void(uint16_t)>> current_item_set_callback_vector;

	/*****************************************
	* Mavlink Message Handlers
	******************************************/
	
	/**
	 * @brief 	Method handle_message_mission_count is used to handle a MISSION COUNT message.
	 * @details	This method is provided so it can be called as a handler every time a MISSION 
	 * 			COUNT message is received. Upon receiving a MISSION COUNT message the class will 
	 * 			send messages MISSION ITEM INT messages in order to retrieve and store all items 
	 * 			of the advertised mission. 
	 * @param 	msg	mavlink_message_t containing the MISSION COUNT message.
	 */
	void handle_message_mission_count(mavlink_message_t msg) {
		// If the message ID is not a MISSION COUNT, print a warning.
		if (msg.msgid != MAVLINK_MSG_ID_MISSION_COUNT) {
			std::cout << format_message("Message is not a MISSION COUNT message so it will be ignored: \n" + std::to_string(msg.msgid), "WARNING");
			return;
		}

		// Get the type of the mission
		uint8_t mission_type = mavlink_msg_mission_count_get_mission_type(&msg);

		// Lock access to the mission items before accessing them.
		std::unique_lock mission_items_lock(mission_items_mutex);

		// Initialise the download maps in preparation for downloading the mission.
		mission_in_progress_buffer[mission_type] = std::map<uint16_t, mavlink_mission_item_int_t>();
		mission_current_download_count_map[mission_type] = mavlink_msg_mission_count_get_count(&msg);
		mission_current_download_item_map[mission_type] = 0;

		mission_items_lock.unlock();

		// Create buffers to send a request for the first mission item.
		mavlink_message_t request_int;
		std::vector<char> buffer_vector = std::vector<char>();

		// Pack a mission item int message for the first mission item that was advertised.
		uint16_t length = mavlink_msg_mission_request_int_pack(
			system_id, component_id, 
			&request_int,
			msg.sysid, msg.compid,
			mission_current_download_item_map[mission_type],
			mission_type);

		uint8_t *buffer_uint8 = (uint8_t *)malloc(length * 2);

		length = mavlink_msg_to_send_buffer(buffer_uint8, &request_int);

		// Copy the byte array to the vector to send to the socket.
		buffer_vector.insert(buffer_vector.end(), &buffer_uint8[0], &buffer_uint8[length]);
		
		// Try sending the vector using the socket.
		try {
			component_socket->send(buffer_vector);
		}
		// Catch and print any errors that occur.
		catch (std::runtime_error e) {
			std::cout << format_message("Error while sending response to mission count: \n" + std::string(e.what()));
		}
	}

	/**
	 * @brief 	Method handle_message_mission_item_int is used to handle a MISSION ITEM INT message. 
	 * @details	This method is provided so it can be called as a handler every time a MISSION 
	 * 			ITEM INT message is received. Upon receiving a MISSION ITEM INT message the class will 
	 * 			store the received mission item then request the next mission item until all items 
	 * 			have been received, according to the download maps configured by handle_message_mission_count.
	 * @param 	msg	mavlink_message_t containing the MISSION ITEM INT message.
	 */
	void handle_message_mission_item_int(mavlink_message_t msg) {
		// If the message ID is not a MISSION ITEM INT, print a warning.
		if (msg.msgid != MAVLINK_MSG_ID_MISSION_ITEM_INT) {
			std::cout << format_message("Message is not a MISSION ITEM INT message so it will be ignored: \n" + std::to_string(msg.msgid), "WARNING");
			return;
		}

		// Get the type of the mission and the mission item number
		uint8_t mission_type = mavlink_msg_mission_item_int_get_mission_type(&msg);
		uint16_t mission_item_number = mavlink_msg_mission_item_int_get_seq(&msg);

		// Lock access to the mission items before accessing them.
		std::unique_lock mission_items_lock(mission_items_mutex);

		// If the mission type is currently being downloaded and the mission item is the current item.
		if (mission_in_progress_buffer.find(mission_type) != mission_in_progress_buffer.end() && 
			mission_item_number == mission_current_download_item_map[mission_type]) 
		{
			// Save the mission item and add it to the in-progress buffer.
			mavlink_mission_item_int_t mission_item;
			mavlink_msg_mission_item_int_decode(&msg, &mission_item);
			mission_in_progress_buffer[mission_type][mission_item_number] = mission_item;
		}

		// If the mission item was the last mission item to be downloaded,
		if (mission_item_number >= mission_current_download_count_map[mission_type] - 1) {

			// Create the buffers to store a MISSION ACK
			mavlink_message_t mission_ack;
			std::vector<char> buffer_vector = std::vector<char>();

			// Pack a MISSION ACK message for the mission type,
			uint16_t length = mavlink_msg_mission_ack_pack(
				system_id, component_id, 
				&mission_ack,
				msg.sysid, msg.compid,
				MAV_MISSION_ACCEPTED,
				mission_type);
			
			uint8_t *buffer_uint8 = (uint8_t *)malloc(length * 2);

			length = mavlink_msg_to_send_buffer(buffer_uint8, &mission_ack);

			// Copy the byte array to the vector to send to the socket.
			buffer_vector.insert(buffer_vector.end(), &buffer_uint8[0], &buffer_uint8[length]);
	
			// Try sending the vector using the socket.
			try {
				component_socket->send(buffer_vector);
			}
			// Catch and print any errors that occur.
			catch (std::runtime_error e) {
				std::cout << format_message("Error while sending mission acknowledgement: \n" + std::string(e.what()));
			}

			// Once the ACK is sent, copy the in-progress download to the downloaded missions map.
			mission_downloaded[mission_type] = mission_in_progress_buffer[mission_type];

			// Remove the in-progress download from the buffer.
			mission_in_progress_buffer.erase(mission_type);

			// If the a callback should be invoked when the mission type has been downloaded fully,
			if (mission_downloaded_callback_map.find(mission_type) != mission_downloaded_callback_map.end()) {
				// Launch the callback in a new thread with the full mission as the argument
				try {
					std::thread callback_thread = std::thread(mission_downloaded_callback_map[mission_type], std::ref(mission_downloaded[mission_type]));
					callback_thread.detach();
				}
				// Catch and print any exceptions that occur.
				catch (std::exception e) {
					std::cout << format_message(
						"Error while executing the mission callback for mission type " + std::to_string(mission_type) + 
						":\n" + std::string(e.what())
					);
				}
			}
		}

		// Else if the mission item was not the last one to be downloaded,
		else {
			// Increment the mission item number.
			mission_current_download_item_map[mission_type] += 1;

			// Create the buffers to store a MISSION REQUEST INT message
			mavlink_message_t request_int;
			std::vector<char> buffer_vector = std::vector<char>();

			// Pack a MISSION REQUEST INT message for the next mission item
			uint16_t length = mavlink_msg_mission_request_int_pack(
				system_id, component_id, 
				&request_int,
				msg.sysid, msg.compid,
				mission_current_download_item_map[mission_type],
				mission_type);
			
			uint8_t *buffer_uint8 = (uint8_t *)malloc(length * 2);

			length = mavlink_msg_to_send_buffer(buffer_uint8, &request_int);

			// Copy the byte array to the vector to send to the socket.
			buffer_vector.insert(buffer_vector.end(), &buffer_uint8[0], &buffer_uint8[length]);
	
			// Try sending the vector using the socket.
			try {
				component_socket->send(buffer_vector);
			}
			// Catch and print any errors that occur.
			catch (std::runtime_error e) {
				std::cout << format_message("Error while sending request for next mission item: \n" + std::string(e.what()));
			}
		}
	}


	/**
	 * @brief 	Method handle_message_mission_set_current is used to handle a MISSION SET CURRENT message.
	 * @details	This method is provided so it can be called as a handler every time a MISSION 
	 * 			SET CURRENT message is received. Upon receiving a MISSION SET CURRENT message the class 
	 * 			will broadcast a MISSION CURRENT message in order to confirm that the current item was set.
	 * @param 	msg	mavlink_message_t containing the MISSION SET CURRENT message.
	 */
	void handle_message_mission_set_current(mavlink_message_t msg) {
		// If the message ID is not a MISSION SET CURRENT, print a warning.
		if (msg.msgid != MAVLINK_MSG_ID_MISSION_SET_CURRENT) {
			std::cout << format_message("Message is not a MISSION SET CURRENT message so it will be ignored: \n" + std::to_string(msg.msgid), "WARNING");
			return;
		}

		uint8_t target_system_id = mavlink_msg_mission_set_current_get_target_system(&msg);
		uint8_t target_component_id = mavlink_msg_mission_set_current_get_target_component(&msg);
		uint16_t sequence_number = mavlink_msg_mission_set_current_get_seq(&msg);

		// If the target in the message matches the system and component id,
		if (target_component_id == component_id && target_system_id == system_id) {
			// Create buffers to send a MISSION CURRENT acknowledgement.
			mavlink_message_t mission_current_ack;
			std::vector<char> buffer_vector = std::vector<char>();

			// Lock access to the mission items before accessing them.
			std::unique_lock mission_items_lock(mission_items_mutex);

			// Set the current sequence number that was retrieved from the message
			current_sequence_number = sequence_number;

			// Pack a mission current acknowledgement with the new current sequence number.
			uint16_t length = mavlink_msg_mission_current_pack(
				system_id, component_id, 
				&mission_current_ack,
				current_sequence_number
			);

			// Unlock access to the mission items after accessing them
			mission_items_lock.unlock();

			// Pack the message into a buffer.
			uint8_t *buffer_uint8 = (uint8_t *)malloc(length * 2);

			length = mavlink_msg_to_send_buffer(buffer_uint8, &mission_current_ack);

			// Copy the byte array to the vector to send to the socket.
			buffer_vector.insert(buffer_vector.end(), &buffer_uint8[0], &buffer_uint8[length]);
			
			// Try sending the vector using the socket.
			try {
				component_socket->send(buffer_vector);
			}
			// Catch and print any errors that occur.
			catch (std::runtime_error e) {
				std::cout << format_message("Error while sending acknowledgement to set current: \n" + std::string(e.what()));
			}

			// Invoke each of the callbacks with the current sequence number. 
			for (auto c : current_item_set_callback_vector) {
				c(sequence_number);
			}
		}

	}

	/**
	 * @brief 	Method handle_message_mission_item_reached is used to handle a MISSION ITEM REACHED message.
	 * @details	This method is provided so it can be called as a handler every time a MISSION 
	 * 			ITEM REACHED message is received. Upon receiving a MISSION ITEM REACHED message the class 
	 * 			will update the current sequence number of the mission.
	 * @param 	msg	mavlink_message_t containing the MISSION ITEM REACHED message.
	 */
	void handle_message_mission_item_reached(mavlink_message_t msg) {
		// If the message ID is not a MISSION ITEM REACHED, print a warning.
		if (msg.msgid != MAVLINK_MSG_ID_MISSION_ITEM_REACHED) {
			std::cout << format_message("Message is not a MISSION ITEM REACHED message so it will be ignored: \n" + std::to_string(msg.msgid), "WARNING");
			return;
		}

		uint16_t sequence_number = mavlink_msg_mission_set_current_get_seq(&msg);

		// Lock access to the mission items before accessing them.
		std::unique_lock mission_items_lock(mission_items_mutex);

		// Set the current sequence number that was retrieved from the message
		current_sequence_number = sequence_number;

		// Unlock access to the mission items after accessing them
		mission_items_lock.unlock();

		// Invoke each of the callbacks with the current sequence number. 
		for (auto c : current_item_set_callback_vector) {
			c(sequence_number);
		}
	}


	/**
	 * @brief 	Method handle_message_mission_current is used to handle a MISSION CURRENT message.
	 * @details	This method is provided so it can be called as a handler every time a MISSION 
	 * 			CURRENT message is received. Upon receiving a MISSION CURRENT message the class 
	 * 			will update the current sequence number of the mission, if it differs from the current.
	 * @param 	msg	mavlink_message_t containing the MISSION CURRENT message.
	 */
	void handle_message_mission_current(mavlink_message_t msg) {
		// If the message ID is not a MISSION CURRENT, print a warning.
		if (msg.msgid != MAVLINK_MSG_ID_MISSION_CURRENT) {
			std::cout << format_message("Message is not a MISSION CURRENT message so it will be ignored: \n" + std::to_string(msg.msgid), "WARNING");
			return;
		}

		uint16_t sequence_number = mavlink_msg_mission_current_get_seq(&msg);
		bool updated = false;

		// Lock access to the mission items before accessing them.
		std::unique_lock mission_items_lock(mission_items_mutex);

		if (sequence_number != current_sequence_number) {
			// Set the current sequence number that was retrieved from the message
			current_sequence_number = sequence_number;
			updated = true;
		}

		// Unlock access to the mission items after accessing them
		mission_items_lock.unlock();

		if (updated) {
			// Invoke each of the callbacks with the current sequence number. 
			for (auto c : current_item_set_callback_vector) {
				c(sequence_number);
			}
		}
	}

	/*****************************************
	* Utility Functions
	******************************************/
	std::string format_message(std::string message, std::string level = "ERROR") {		
		return "[Mavlink Mission Helper] (" + level + ")\t\t\t" + message + "\n";
	}
};

#endif /* MAVLINK_HELPER_MISSION_HPP */