// serialtest.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <string>
#include "serial/serial.h"

int main()
{
    serial::Serial relativ;
	relativ.setPort("COM12");
	relativ.setBaudrate(115200);
	relativ.open();
	std::string last_recv;
	try {
		while (relativ.isOpen()) {

			last_recv = relativ.readline();
			std::string raw_str = last_recv;
			if (raw_str.size() > 0) {
				if (raw_str[0] != 0) {
					if (raw_str.size() > 3) {
						bool valid_reading = true;
						float read_vals[4] = { 2,2,2,2 }; // Quat values will never be greater/less than 1,-1
						try {
							size_t pos = 0;
							pos = raw_str.find(',');
							read_vals[0] = std::stof(raw_str.substr(0, pos));
							raw_str.erase(0, pos + 1);

							pos = raw_str.find(',');
							read_vals[1] = std::stof(raw_str.substr(0, pos));
							raw_str.erase(0, pos + 1);

							pos = raw_str.find(',');
							read_vals[2] = std::stof(raw_str.substr(0, pos));
							raw_str.erase(0, pos + 1);

							read_vals[3] = std::stof(raw_str);
						}
						catch (...) {
							valid_reading = false;
						}
						if (valid_reading) {
							std::cout << std::to_string(read_vals[0]) + "," + std::to_string(read_vals[1]) + "," + std::to_string(read_vals[2]) + "," + std::to_string(read_vals[3]) + "\n";
						}
					}
				}
			}
		}

	}
	catch (...) {
		
	}

    std::cout << "Hello World!\n";
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
