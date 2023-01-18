#include "ClientComponent.hpp"

#include <iostream>
#include <vector>

int main() {
	try {
		ClientComponent c1 = ClientComponent(1, 5, 44444);
		// c1.test_connection();
		Sleep(30000);
	}
	catch (std::runtime_error e) {
		std::cout << e.what() << std::endl;
		return -1;
	}
    return 0;
}