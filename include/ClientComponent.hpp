#ifndef CLIENT_COMPONENT_HPP
#define CLIENT_COMPONENT_HPP

#include "Socket/Socket.hpp"

#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>

class ClientComponent {
public:
    ClientComponent(uint8_t system_id, uint8_t component_id, uint32_t port) : 
			system_id(system_id), component_id(component_id), local_port(local_port) 
	{
		component_socket = Socket(local_port);
		
	}


protected:
    uint8_t system_id;
    uint8_t component_id;
    uint32_t local_port;

	Socket component_socket;
};

#endif /* CLIENT_COMPONENT_HPP */