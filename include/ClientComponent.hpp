#ifndef CLIENT_COMPONENT_HPP
#define CLIENT_COMPONENT_HPP

#include <Socket/Socket.hpp>

#include <mavlink.h>

#include <chrono>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

class ClientComponent {
public:
    ClientComponent(
		uint8_t system_id, uint8_t component_id, 
		unsigned short local_port, unsigned int heartbeat_interval_ms = 1000) : 
			system_id(system_id), component_id(component_id),
			local_port(local_port), heartbeat_interval_ms(heartbeat_interval_ms),
			component_socket(local_port)
	{
		close_component = false;
		std::thread heartbeat_thread = std::thread(&ClientComponent::gcs_heatbeat_thread, this);
		heartbeat_thread.detach();
	}

	~ClientComponent() {
		close_component = true;
		std::this_thread::sleep_for(std::chrono::milliseconds(heartbeat_interval_ms * 2));
	}

	void gcs_heatbeat_thread() {
		std::vector<char> buffer_vector = std::vector<char>();
		mavlink_message_t msg;
		uint8_t buffer_uint8[128];
		char *buffer_char;
		int old_length;
		int new_length;

		mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_TYPE_HELICOPTER, MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);
		old_length = mavlink_msg_to_send_buffer(buffer_uint8, &msg);

		buffer_char = (char *)malloc(old_length);

		Socket heartbeat_socket = Socket(local_port + 1);
		heartbeat_socket.configure_remote_host(14550, "0.0.0.0");

		while (!close_component) {
			mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_TYPE_HELICOPTER, MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);
			new_length = mavlink_msg_to_send_buffer(buffer_uint8, &msg);
			
			if (new_length != old_length) {
				old_length = new_length;
				free(buffer_char);
				buffer_char = (char *)malloc(new_length);
			}

			memcpy(buffer_char, buffer_uint8, new_length);
			buffer_vector.insert(buffer_vector.end(), &buffer_char[0], &buffer_char[new_length]);
			
			int bytes_sent = heartbeat_socket.send_to(buffer_vector, 14550);

			buffer_vector.clear();
			std::this_thread::sleep_for(std::chrono::milliseconds(heartbeat_interval_ms));
		}
		free(buffer_char);
	}

	void test_connection() {
		uint8_t buf[2041];
		std::vector<char> vector_buffer = std::vector<char>();
		mavlink_message_t msg;

		mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_TYPE_HELICOPTER, MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);
		int len = mavlink_msg_to_send_buffer(buf, &msg);
		for (int i = 0; i < len; i++) {
			vector_buffer.push_back(char(buf[i]));
		}
		int bytes_sent = component_socket.send_to(vector_buffer, 14550);
		memset(buf, 0, sizeof(buf));
		
		/* Send Status */
		mavlink_msg_sys_status_pack(1, 200, &msg, 0, 0, 0, 500, 11000, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0);
		len = mavlink_msg_to_send_buffer(buf, &msg);
		for (int i = 0; i < len; i++) {
			vector_buffer.push_back(char(buf[i]));
		}
		bytes_sent = component_socket.send_to(vector_buffer, 14550);
		memset(buf, 0, sizeof(buf));
		
		float position[6] = {45.4215, -75.6972, 1000, 0, 0, 0};
		/* Send Local Position */
		mavlink_msg_local_position_ned_pack(1, 200, &msg, 10000000000, 
										position[0], position[1], position[2],
										position[3], position[4], position[5]);
		len = mavlink_msg_to_send_buffer(buf, &msg);
		for (int i = 0; i < len; i++) {
			vector_buffer.push_back(char(buf[i]));
		}
		bytes_sent = component_socket.send_to(vector_buffer, 14550);
		memset(buf, 0, sizeof(buf));
		
		/* Send attitude */
		mavlink_msg_attitude_pack(1, 200, &msg, 10000000000, 1.2, 1.7, 3.14, 0.01, 0.02, 0.03);
		len = mavlink_msg_to_send_buffer(buf, &msg);
		for (int i = 0; i < len; i++) {
			vector_buffer.push_back(char(buf[i]));
		}
		bytes_sent = component_socket.send_to(vector_buffer, 14550);
		memset(buf, 0, sizeof(buf));
		std::vector<char> received_data = component_socket.receive(sizeof(buf));
		if (received_data.size() > 0)
      	{
			// Something received - print out all bytes and parse packet
			mavlink_message_t msg;
			mavlink_status_t status;

			char *received_buffer = (char *)malloc(received_data.size());
			memcpy(received_buffer, received_data.data(), received_data.size());
			int received_buffer_size = received_data.size();

			printf("Bytes Received: %d\nDatagram: ", received_data.size());

			char temp;
			for (int i = 0; i < received_data.size(); i++)
			{
				temp = received_buffer[i];
				printf("%02x ", (unsigned char)temp);
				if (mavlink_parse_char(MAVLINK_COMM_0, received_buffer[i], &msg, &status))
				{
					// Packet received
					printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", msg.sysid, msg.compid, msg.len, msg.msgid);
				}
			}
			printf("\n");
		}
		memset(buf, 0, sizeof(buf));
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}

protected:
    uint8_t system_id;
    uint8_t component_id;
    unsigned short local_port;

	Socket component_socket;

	bool close_component;

	unsigned int heartbeat_interval_ms;
};

#endif /* CLIENT_COMPONENT_HPP */