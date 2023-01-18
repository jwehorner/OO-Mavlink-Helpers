#include <Socket/Socket.hpp>

#include <iostream>
#include <vector>

int main() {
	try {
		Socket s = Socket(56767);
		std::vector<char> data = s.receive(sizeof(float));
		for (char b : data) {
			std::cout << std::hex << (int)b;
		}
		std::cout << std::endl;
	}
	catch (std::runtime_error e) {
		std::cout << e.what() << std::endl;
		return -1;
	}
    return 0;
}