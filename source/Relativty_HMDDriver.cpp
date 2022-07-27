// Copyright (C) 2020  Max Coutte, Gabriel Combe
// Copyright (C) 2020  Relativty.com
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#pragma comment(lib, "Ws2_32.lib")
#pragma comment (lib, "Setupapi.lib")
#pragma comment(lib, "User32.lib")

#include <atomic>
#include <WinSock2.h>
#include <Windows.h>
#include "hidapi/hidapi.h"
#include "openvr_driver.h"
#include "serial/serial.h"

#include "driverlog.h"

#include "Relativty_HMDDriver.hpp"
#include "Relativty_ServerDriver.hpp"
#include "Relativty_EmbeddedPython.h"
#include "Relativty_components.h"
#include "Relativty_base_device.h"

#include <iostream>
#include <librealsense2/rs.hpp>
#include <spectacularAI/realsense/plugin.hpp>

#include <string>

#include <vector>
#define BUFLEN 512
#define PORT 50000

#define SIO_UDP_CONNRESET _WSAIOW(IOC_VENDOR, 12)


inline vr::HmdQuaternion_t HmdQuaternion_Init(double w, double x, double y, double z) {
	vr::HmdQuaternion_t quat;
	quat.w = w;
	quat.x = x;
	quat.y = y;
	quat.z = z;
	return quat;
}

inline void Normalize(float norma[3], float v[3], float max[3], float min[3], int up, int down, float scale[3], float offset[3]) {
	for (int i = 0; i < 4; i++) {
		norma[i] = (((up - down) * ((v[i] - min[i]) / (max[i] - min[i])) + down) / scale[i])+ offset[i];
	}
}

vr::EVRInitError Relativty::HMDDriver::Activate(uint32_t unObjectId) {
	RelativtyDevice::Activate(unObjectId);
	this->setProperties();

	int result;
	DriverLog("SERIAL: %s.\n", COMPORT);
	DriverLog("IS MPU SERIAL: %d\n", isMPUSerial);
	this->isMPUSerial = true;



	this->update_pose_thread_worker = std::thread(&Relativty::HMDDriver::update_pose_threaded2, this);

	return vr::VRInitError_None;

}


void Relativty::HMDDriver::Deactivate() {
	//this->retrieve_quaternion_isOn = false;
	//this->retrieve_quaternion_thread_worker.join();
	//hid_close(this->handle);
	//hid_exit();

	//this->retrieve_vector_isOn = false;
	//closesocket(this->sock);
	//this->retrieve_vector_thread_worker.join();
	WSACleanup();

	RelativtyDevice::Deactivate();
	this->update_pose_thread_worker.join();

	Relativty::ServerDriver::Log("Thread0: all threads exit correctly \n");
}

void Relativty::HMDDriver::update_pose_threaded() {
	Relativty::ServerDriver::Log("Thread2: successfully started\n");

	float normalize_min[3]{ this->normalizeMinX, this->normalizeMinY, this->normalizeMinZ };
	float normalize_max[3]{ this->normalizeMaxX, this->normalizeMaxY, this->normalizeMaxZ };
	float scales_coordinate_meter[3]{ this->scalesCoordinateMeterX, this->scalesCoordinateMeterY, this->scalesCoordinateMeterZ };
	float offset_coordinate[3] = { this->offsetCoordinateX, this->offsetCoordinateY, this->offsetCoordinateZ };
	float coordinate[3] = { 0, 0, 0 };
	float rotation[4] = { 0, 0, 0, 0 };

	Relativty::ServerDriver::Log("UDP SERVER: Initialising UDP COMMS.\n");
	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
	{
		Relativty::ServerDriver::Log("UDP SERVER: Failed to Init UDP\n");
		return;
	}
	Relativty::ServerDriver::Log("UDP SERVER:  Initialised.\n");

	// create a socket

	if ((server_socket = socket(AF_INET, SOCK_DGRAM, 0)) == INVALID_SOCKET)
	{
		Relativty::ServerDriver::Log("UDP SERVER: Could not create socket.\n");
		return;
	}
	Relativty::ServerDriver::Log("UDP SERVER: Socket created.\n");
	WSAIoctl(server_socket, SIO_UDP_CONNRESET, &bNewBehavior, sizeof bNewBehavior, NULL, 0, &dwBytesReturned, NULL, NULL);
	server.sin_family = AF_INET;
	server.sin_addr.s_addr = INADDR_ANY;
	server.sin_port = htons(PORT);
	// bind
	if (bind(server_socket, (sockaddr*)&server, sizeof(server)) == SOCKET_ERROR)
	{
		Relativty::ServerDriver::Log("UDP SERVER: Bind failed\n");
		return;
	}
	puts("UDP SERVER: Bind done.");
	this->serverNotReady = false;
	Relativty::ServerDriver::Log("UDP SERVER: Waiting for incoming connections...\n");

	while (m_unObjectId != vr::k_unTrackedDeviceIndexInvalid) {
		
		Relativty::ServerDriver::Log("UDP SERVER: Waiting for data...");
		fflush(stdout);
		char message[BUFLEN] = {};

		// try to receive some data, this is a blocking call
		int message_len;
		int slen = sizeof(sockaddr_in);
		if (message_len = recvfrom(server_socket, message, BUFLEN, 0, (sockaddr*)&client, &slen) == SOCKET_ERROR)
		{
			Relativty::ServerDriver::Log("UDP SERVER: recvfrom() failed");
			exit(0);
		}
		std::string messageString = message;
		if (isspace(messageString[0])) { messageString.erase(0, 1); }


		std::string space_delimiter = " ";
		std::vector<std::string> words{};

		size_t pos = 0;

		while ((pos = messageString.find(space_delimiter)) != std::string::npos) {
			words.push_back(messageString.substr(0, pos));
			messageString.erase(0, pos + space_delimiter.length());
		}
		
		
		coordinate[0] = std::stof(words[1]);
		coordinate[1] = std::stof(words[2]);
		coordinate[2] = std::stof(words[0]);

		rotation[0] = std::stof(words[3]);
		rotation[1] = std::stof(words[5]);
		rotation[2] = std::stof(words[6]);
		rotation[3] = std::stof(words[4]);
				
		Relativty::ServerDriver::Log("UDP SERVER: " + words[0] + "," + words[1] + "," + words[2] + "," + words[3] + "," + words[4] + "," + words[5] + "," + words[6] + "\n");

		this->vector_xyz[0] = coordinate[0];
		this->vector_xyz[1] = coordinate[1];
		this->vector_xyz[2] = coordinate[2];
		this->quat[0] = rotation[0];
		this->quat[1] = rotation[2];
		this->quat[2] = rotation[3];
		this->quat[3] = rotation[1];
		
		this->calibrate_quaternion();

		m_Pose.qRotation.w = this->quat[0];
		m_Pose.qRotation.x = this->quat[1];
		m_Pose.qRotation.y = this->quat[2];
		m_Pose.qRotation.z = this->quat[3];

		m_Pose.vecPosition[0] = this->vector_xyz[0];
		m_Pose.vecPosition[1] = this->vector_xyz[1];
		m_Pose.vecPosition[2] = this->vector_xyz[2];

		vr::VRServerDriverHost()->TrackedDevicePoseUpdated(m_unObjectId, m_Pose, sizeof(vr::DriverPose_t));

	}
	Relativty::ServerDriver::Log("Thread2: successfully stopped\n");
}


void Relativty::HMDDriver::update_pose_threaded2()
{
	spectacularAI::rsPlugin::Pipeline vioPipeline(config);
	rs2::device_list devices = rsContext.query_devices();
	if (devices.size() != 1) {
		DriverLog("Connect exactly one RealSense device.\n");
	}
	rs2::device device = devices.front();
	vioPipeline.configureDevice(device);

	rs2::config rsConfig;
	vioPipeline.configureStreams(rsConfig);
	auto vioSession = vioPipeline.startSession(rsConfig);

	while (m_unObjectId != vr::k_unTrackedDeviceIndexInvalid)
	{
		auto vioOut = vioSession->waitForOutput();
		//std::cout << vioOut->asJson() << std::endl;
		DriverLog("%s\n", vioOut->asJson());
	}
}

void Relativty::HMDDriver::calibrate_quaternion() {
	if ((0x01 & GetAsyncKeyState(0x52)) != 0) {
		qconj[0].store(quat[0]);
		qconj[1].store(-1 * quat[1]);
		qconj[2].store(-1 * quat[2]);
		qconj[3].store(-1 * quat[3]);
	}
	float qres[4];

	qres[0] = qconj[0] * quat[0] - qconj[1] * quat[1] - qconj[2] * quat[2] - qconj[3] * quat[3];
	qres[1] = qconj[0] * quat[1] + qconj[1] * quat[0] + qconj[2] * quat[3] - qconj[3] * quat[2];
	qres[2] = qconj[0] * quat[2] - qconj[1] * quat[3] + qconj[2] * quat[0] + qconj[3] * quat[1];
	qres[3] = qconj[0] * quat[3] + qconj[1] * quat[2] - qconj[2] * quat[1] + qconj[3] * quat[0];

	this->quat[0] = qres[0];
	this->quat[1] = qres[1];
	this->quat[2] = qres[2];
	this->quat[3] = qres[3];
}

/*
void Relativty::HMDDriver::dummy_quaternion_thread_func()
{
	//nothing :c
};

void Relativty::HMDDriver::retrieve_device_quaternion_packet_threaded() {
	uint8_t packet_buffer[64];
	int16_t quaternion_packet[4];
	//this struct is for mpu9250 support
	#pragma pack(push, 1)
	struct pak {
		uint8_t id;
		float quat[4];
		uint8_t rest[47];
	};
	#pragma pack(pop)
	int result;
	Relativty::ServerDriver::Log("Thread1: successfully started\n");
	while (this->retrieve_quaternion_isOn) {

		if (!isMPUSerial)
		{
			result = hid_read(this->handle, packet_buffer, 64); //Result should be greater than 0.
			if (result > 0) {


				if (m_bIMUpktIsDMP) {

					quaternion_packet[0] = ((packet_buffer[1] << 8) | packet_buffer[2]);
					quaternion_packet[1] = ((packet_buffer[5] << 8) | packet_buffer[6]);
					quaternion_packet[2] = ((packet_buffer[9] << 8) | packet_buffer[10]);
					quaternion_packet[3] = ((packet_buffer[13] << 8) | packet_buffer[14]);
					this->quat[0] = static_cast<float>(quaternion_packet[0]) / 16384.0f;
					this->quat[1] = static_cast<float>(quaternion_packet[1]) / 16384.0f;
					this->quat[2] = static_cast<float>(quaternion_packet[2]) / 16384.0f;
					this->quat[3] = static_cast<float>(quaternion_packet[3]) / 16384.0f;

					float qres[4];
					qres[0] = quat[0];
					qres[1] = quat[1];
					qres[2] = -1 * quat[2];
					qres[3] = -1 * quat[3];

					this->quat[0] = qres[0];
					this->quat[1] = qres[1];
					this->quat[2] = qres[2];
					this->quat[3] = qres[3];

					this->calibrate_quaternion();

					this->new_quaternion_avaiable = true;

				}
				else {

					pak* recv = (pak*)packet_buffer;
					this->quat[0] = recv->quat[0];
					this->quat[1] = recv->quat[1];
					this->quat[2] = recv->quat[2];
					this->quat[3] = recv->quat[3];

					this->calibrate_quaternion();

					this->new_quaternion_avaiable = true;

				}


			}
			else {
				Relativty::ServerDriver::Log("Thread1: Issue while trying to read USB\n");
			}

		}
		else
		{
			//serial::Serial relativ;
			std::string last_recv;
			try {
				while (relativ.isOpen()) {
					if (last_recv.size() > 0 && last_recv[0] != 0) {
						if (GetAsyncKeyState(VK_SHIFT) != 0 && GetAsyncKeyState(VK_CONTROL) != 0 && GetAsyncKeyState(0x49) != 0) {
							relativ.write("C\n");
						}
					}
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
									this->quat[0] = read_vals[0];
									this->quat[1] = read_vals[1];
									this->quat[2] = read_vals[2];
									this->quat[3] = read_vals[3];

									this->calibrate_quaternion();

									this->new_quaternion_avaiable = true;

								}
							}
						}
						else if (raw_str[0] == 'C') {
							if (raw_str.size() > 3) {
								std::string raw_str_c = raw_str.substr(2, raw_str.size()); // Remove "C:"
								Relativty::ServerDriver::Log("Thread1: Calibration: " + raw_str_c);
							}
						}
						else if (raw_str[0] == 'D') {
							if (raw_str.size() > 3) {
								std::string raw_str_d = raw_str.substr(2, raw_str.size()); // Remove "D:"
								Relativty::ServerDriver::Log("Thread1: Info: " + raw_str_d);
							}
						}
						else {
							// invalid response...
						}
					}
				}

			}
			catch (...) {
				Relativty::ServerDriver::Log("Thread1: Connection with SERIAL lost!");
			}
		}

	}
	Relativty::ServerDriver::Log("Thread1: successfully stopped\n");
}
*/
/*
void Relativty::HMDDriver::retrieve_client_vector_packet_threaded_UDP()
{
	BOOL bNewBehavior = FALSE;
	DWORD dwBytesReturned = 0;
	sockaddr_in server, client;
	WSADATA wsa;

	float normalize_min[3]{ this->normalizeMinX, this->normalizeMinY, this->normalizeMinZ };
	float normalize_max[3]{ this->normalizeMaxX, this->normalizeMaxY, this->normalizeMaxZ };
	float scales_coordinate_meter[3]{ this->scalesCoordinateMeterX, this->scalesCoordinateMeterY, this->scalesCoordinateMeterZ };
	float offset_coordinate[3] = { this->offsetCoordinateX, this->offsetCoordinateY, this->offsetCoordinateZ };

	float coordinate[3]{ 0, 0, 0 };
	float coordinate_normalized[3];



	Relativty::ServerDriver::Log("UDP SERVER: Initialising UDP COMMS.\n");
	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
	{
		Relativty::ServerDriver::Log("UDP SERVER: Failed to Init UDP\n");
		return;
	}
	Relativty::ServerDriver::Log("UDP SERVER:  Initialised.\n");

	// create a socket
	SOCKET server_socket;
	if ((server_socket = socket(AF_INET, SOCK_DGRAM, 0)) == INVALID_SOCKET)
	{
		Relativty::ServerDriver::Log("UDP SERVER: Could not create socket.\n");
		return;
	}
	Relativty::ServerDriver::Log("UDP SERVER: Socket created.\n");
	WSAIoctl(server_socket, SIO_UDP_CONNRESET, &bNewBehavior, sizeof bNewBehavior, NULL, 0, &dwBytesReturned, NULL, NULL);
	server.sin_family = AF_INET;
	server.sin_addr.s_addr = INADDR_ANY;
	server.sin_port = htons(PORT);

	// bind
	if (bind(server_socket, (sockaddr*)&server, sizeof(server)) == SOCKET_ERROR)
	{
		Relativty::ServerDriver::Log("UDP SERVER: Bind failed\n");
		return;
	}
	puts("UDP SERVER: Bind done.");
	this->serverNotReady = false;
	Relativty::ServerDriver::Log("UDP SERVER: Waiting for incoming connections...\n");

	while (this->retrieve_vector_isOn)
	{
		Relativty::ServerDriver::Log("UDP SERVER: Waiting for data...");
		fflush(stdout);
		char message[BUFLEN] = {};

		// try to receive some data, this is a blocking call
		int message_len;
		int slen = sizeof(sockaddr_in);
		if (message_len = recvfrom(server_socket, message, BUFLEN, 0, (sockaddr*)&client, &slen) == SOCKET_ERROR)
		{
			Relativty::ServerDriver::Log("UDP SERVER: recvfrom() failed");
			exit(0);
		}

		// print details of the client/peer and the data received
		//printf("Received packet from %s:%d\n", inet_ntoa(client.sin_addr), ntohs(client.sin_port));
		std::string messageString = message;
		if (isspace(messageString[0])) { messageString.erase(0, 1); }
		//printf("Data: %s\n", messageString);
		//std::cout << "DATA: " << messageString << "\n";
		std::string space_delimiter = " ";
		std::vector<std::string> words{};

		size_t pos = 0;

		while ((pos = messageString.find(space_delimiter)) != std::string::npos) {
			words.push_back(messageString.substr(0, pos));
			messageString.erase(0, pos + space_delimiter.length());
		}

		coordinate[0] = std::stof(words[0]);
		coordinate[1] = std::stof(words[1]);
		coordinate[2] = std::stof(words[2]);

		Relativty::ServerDriver::Log("UDP SERVER: " + words[0] + "," + words[1] + "," + words[2] + "\n");
		//Relativty::ServerDriver::Log("UDP SERVER: " + words[0] + "," + words[1] + "," + words[2] + words[3] + "," + words[4] + "," + words[5] + "," + words[6] + "\n");

		//Normalize(coordinate_normalized, coordinate, normalize_max, normalize_min, this->upperBound, this->lowerBound, scales_coordinate_meter, offset_coordinate);
		//this->vector_xyz[0] = coordinate_normalized[1];
		//this->vector_xyz[1] = coordinate_normalized[2];
		//this->vector_xyz[2] = coordinate_normalized[0];
		this->vector_xyz[0] = coordinate[1];
		this->vector_xyz[1] = coordinate[2];
		this->vector_xyz[2] = coordinate[0];
		this->new_vector_avaiable = true;

		if (sendto(server_socket, message, strlen(message), 0, (sockaddr*)&client, sizeof(sockaddr_in)) == SOCKET_ERROR)
		{
			Relativty::ServerDriver::Log("sendto() failed");
			return;
		}
	}
}
*/
/*
void Relativty::HMDDriver::retrieve_client_vector_packet_threaded_UDP_REALSENSE()
{
	BOOL bNewBehavior = FALSE;
	DWORD dwBytesReturned = 0;
	sockaddr_in server, client;
	WSADATA wsa;

	float normalize_min[3]{ this->normalizeMinX, this->normalizeMinY, this->normalizeMinZ };
	float normalize_max[3]{ this->normalizeMaxX, this->normalizeMaxY, this->normalizeMaxZ };
	float scales_coordinate_meter[3]{ this->scalesCoordinateMeterX, this->scalesCoordinateMeterY, this->scalesCoordinateMeterZ };
	float offset_coordinate[3] = { this->offsetCoordinateX, this->offsetCoordinateY, this->offsetCoordinateZ };

	float coordinate[3]{ 0, 0, 0 };
	float rotation[4]{ 0, 0, 0, 0 };
	float coordinate_normalized[3];



	Relativty::ServerDriver::Log("UDP SERVER: Initialising UDP COMMS.\n");
	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
	{
		Relativty::ServerDriver::Log("UDP SERVER: Failed to Init UDP\n");
		return;
	}
	Relativty::ServerDriver::Log("UDP SERVER:  Initialised.\n");

	// create a socket
	SOCKET server_socket;
	if ((server_socket = socket(AF_INET, SOCK_DGRAM, 0)) == INVALID_SOCKET)
	{
		Relativty::ServerDriver::Log("UDP SERVER: Could not create socket.\n");
		return;
	}
	Relativty::ServerDriver::Log("UDP SERVER: Socket created.\n");
	WSAIoctl(server_socket, SIO_UDP_CONNRESET, &bNewBehavior, sizeof bNewBehavior, NULL, 0, &dwBytesReturned, NULL, NULL);
	server.sin_family = AF_INET;
	server.sin_addr.s_addr = INADDR_ANY;
	server.sin_port = htons(PORT);

	// bind
	if (bind(server_socket, (sockaddr*)&server, sizeof(server)) == SOCKET_ERROR)
	{
		Relativty::ServerDriver::Log("UDP SERVER: Bind failed\n");
		return;
	}
	puts("UDP SERVER: Bind done.");
	this->serverNotReady = false;
	Relativty::ServerDriver::Log("UDP SERVER: Waiting for incoming connections...\n");

	while (this->retrieve_vector_isOn)
	{
		Relativty::ServerDriver::Log("UDP SERVER: Waiting for data...");
		fflush(stdout);
		char message[BUFLEN] = {};

		// try to receive some data, this is a blocking call
		int message_len;
		int slen = sizeof(sockaddr_in);
		if (message_len = recvfrom(server_socket, message, BUFLEN, 0, (sockaddr*)&client, &slen) == SOCKET_ERROR)
		{
			Relativty::ServerDriver::Log("UDP SERVER: recvfrom() failed");
			exit(0);
		}

		std::string messageString = message;
		if (isspace(messageString[0])) { messageString.erase(0, 1); }


		std::string space_delimiter = " ";
		std::vector<std::string> words{};

		size_t pos = 0;

		while ((pos = messageString.find(space_delimiter)) != std::string::npos) {
			words.push_back(messageString.substr(0, pos));
			messageString.erase(0, pos + space_delimiter.length());
		}

		coordinate[0] = std::stof(words[1]);
		coordinate[1] = std::stof(words[2]);
		coordinate[2] = std::stof(words[0]);

		rotation[0] = std::stof(words[3]);
		rotation[1] = std::stof(words[5]);
		rotation[2] = std::stof(words[6]);
		rotation[3] = std::stof(words[4]);

		Relativty::ServerDriver::Log("UDP SERVER: " + words[0] + "," + words[1] + "," + words[2] + words[3] + "," + words[4] + "," + words[5] + "," + words[6] + "\n");

		this->vector_xyz[0] = coordinate[0];
		this->vector_xyz[1] = coordinate[1];
		this->vector_xyz[2] = coordinate[2];
		this->quat[0] = rotation[0];
		this->quat[1] = rotation[2];
		this->quat[2] = rotation[3];
		this->quat[3] = rotation[1];

		this->calibrate_quaternion();

		this->new_quaternion_avaiable = true;
		this->new_vector_avaiable = true;

		if (sendto(server_socket, message, strlen(message), 0, (sockaddr*)&client, sizeof(sockaddr_in)) == SOCKET_ERROR)
		{
			Relativty::ServerDriver::Log("sendto() failed");
			return;
		}
	}
}
*/

/*
void Relativty::HMDDriver::retrieve_client_vector_packet_threaded() {
	WSADATA wsaData;
	struct sockaddr_in server, client;
	int addressLen;
	int receiveBufferLen = 12;
	char receiveBuffer[12];
	int resultReceiveLen;

	float normalize_min[3]{ this->normalizeMinX, this->normalizeMinY, this->normalizeMinZ};
	float normalize_max[3]{ this->normalizeMaxX, this->normalizeMaxY, this->normalizeMaxZ};
	float scales_coordinate_meter[3]{ this->scalesCoordinateMeterX, this->scalesCoordinateMeterY, this->scalesCoordinateMeterZ};
	float offset_coordinate[3] = { this->offsetCoordinateX, this->offsetCoordinateY, this->offsetCoordinateZ};

	float coordinate[3]{ 0, 0, 0 };
	float coordinate_normalized[3];

	Relativty::ServerDriver::Log("Thread3: Initialising Socket.\n");
	if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
		Relativty::ServerDriver::Log("Thread3: Failed. Error Code: " + WSAGetLastError());
		return;
	}
	Relativty::ServerDriver::Log("Thread3: Socket successfully initialised.\n");

	if ((this->sock = socket(AF_INET, SOCK_STREAM, 0)) == INVALID_SOCKET)
		Relativty::ServerDriver::Log("Thread3: could not create socket: " + WSAGetLastError());
	Relativty::ServerDriver::Log("Thread3: Socket created.\n");

	server.sin_family = AF_INET;
	server.sin_port = htons(50000);
	server.sin_addr.s_addr = INADDR_ANY;

	if (bind(this->sock, (struct sockaddr*) & server, sizeof(server)) == SOCKET_ERROR)
		Relativty::ServerDriver::Log("Thread3: Bind failed with error code: " + WSAGetLastError());
	Relativty::ServerDriver::Log("Thread3: Bind done \n");

	listen(this->sock, 1);

	this->serverNotReady = false;

	Relativty::ServerDriver::Log("Thread3: Waiting for incoming connections...\n");
	addressLen = sizeof(struct sockaddr_in);
	this->sock_receive = accept(this->sock, (struct sockaddr*) & client, &addressLen);
	if (this->sock_receive == INVALID_SOCKET)
		Relativty::ServerDriver::Log("Thread3: accept failed with error code: " + WSAGetLastError());
	Relativty::ServerDriver::Log("Thread3: Connection accepted");

	Relativty::ServerDriver::Log("Thread3: successfully started\n");
	while (this->retrieve_vector_isOn) {
		resultReceiveLen = recv(this->sock_receive, receiveBuffer, receiveBufferLen, NULL);
		if (resultReceiveLen > 0) {
			coordinate[0] = *(float*)(receiveBuffer);
			coordinate[1] = *(float*)(receiveBuffer + 4);
			coordinate[2] = *(float*)(receiveBuffer + 8);

			Normalize(coordinate_normalized, coordinate, normalize_max, normalize_min, this->upperBound, this->lowerBound, scales_coordinate_meter, offset_coordinate);

			this->vector_xyz[0] = coordinate_normalized[1];
			this->vector_xyz[1] = coordinate_normalized[2];
			this->vector_xyz[2] = coordinate_normalized[0];
			this->new_vector_avaiable = true;
		}
	}
	Relativty::ServerDriver::Log("Thread3: successfully stopped\n");
}
*/


Relativty::HMDDriver::HMDDriver(std::string myserial):RelativtyDevice(myserial, "akira_") {
	// keys for use with the settings API
	static const char* const Relativty_hmd_section = "Relativty_hmd";

	// openvr api stuff
	m_sRenderModelPath = "{Relativty}/rendermodels/generic_hmd";
	m_sBindPath = "{Relativty}/input/relativty_hmd_profile.json";

	m_spExtDisplayComp = std::make_shared<Relativty::RelativtyExtendedDisplayComponent>();

	// not openvr api stuff
	Relativty::ServerDriver::Log("Loading Settings\n");
	this->IPD = vr::VRSettings()->GetFloat(Relativty_hmd_section, "IPDmeters");
	this->SecondsFromVsyncToPhotons = vr::VRSettings()->GetFloat(Relativty_hmd_section, "secondsFromVsyncToPhotons");
	this->DisplayFrequency = vr::VRSettings()->GetFloat(Relativty_hmd_section, "displayFrequency");

	this->start_tracking_server = vr::VRSettings()->GetBool(Relativty_hmd_section, "startTrackingServer");
	this->upperBound = vr::VRSettings()->GetFloat(Relativty_hmd_section, "upperBound");
	this->lowerBound = vr::VRSettings()->GetFloat(Relativty_hmd_section, "lowerBound");
	this->normalizeMinX = vr::VRSettings()->GetFloat(Relativty_hmd_section, "normalizeMinX");
	this->normalizeMinY = vr::VRSettings()->GetFloat(Relativty_hmd_section, "normalizeMinY");
	this->normalizeMinZ = vr::VRSettings()->GetFloat(Relativty_hmd_section, "normalizeMinZ");
	this->normalizeMaxX = vr::VRSettings()->GetFloat(Relativty_hmd_section, "normalizeMaxX");
	this->normalizeMaxY = vr::VRSettings()->GetFloat(Relativty_hmd_section, "normalizeMaxY");
	this->normalizeMaxZ = vr::VRSettings()->GetFloat(Relativty_hmd_section, "normalizeMaxZ");
	this->scalesCoordinateMeterX = vr::VRSettings()->GetFloat(Relativty_hmd_section, "scalesCoordinateMeterX");
	this->scalesCoordinateMeterY = vr::VRSettings()->GetFloat(Relativty_hmd_section, "scalesCoordinateMeterY");
	this->scalesCoordinateMeterZ = vr::VRSettings()->GetFloat(Relativty_hmd_section, "scalesCoordinateMeterZ");
	this->offsetCoordinateX = vr::VRSettings()->GetFloat(Relativty_hmd_section, "offsetCoordinateX");
	this->offsetCoordinateY = vr::VRSettings()->GetFloat(Relativty_hmd_section, "offsetCoordinateY");
	this->offsetCoordinateZ = vr::VRSettings()->GetFloat(Relativty_hmd_section, "offsetCoordinateZ");

	this->m_iPid = vr::VRSettings()->GetInt32(Relativty_hmd_section, "hmdPid");
	this->m_iVid = vr::VRSettings()->GetInt32(Relativty_hmd_section, "hmdVid");

	this->m_bIMUpktIsDMP = vr::VRSettings()->GetBool(Relativty_hmd_section, "hmdIMUdmpPackets");

	this->isMPUSerial = vr::VRSettings()->GetBool(Relativty_hmd_section, "isMPUSerial");


	char buffer[1024];
	vr::VRSettings()->GetString(Relativty_hmd_section, "PyPath", buffer, sizeof(buffer));
	this->PyPath = buffer;
	buffer[0] = 0;
	vr::VRSettings()->GetString(Relativty_hmd_section, "COMPORT", buffer, sizeof(buffer));
	this->COMPORT = buffer;

	// this is a bad idea, this should be set by the tracking loop
	m_Pose.result = vr::TrackingResult_Running_OK;
}

inline void Relativty::HMDDriver::setProperties() {
	vr::VRProperties()->SetFloatProperty(m_ulPropertyContainer, vr::Prop_UserIpdMeters_Float, this->IPD);
	vr::VRProperties()->SetFloatProperty(m_ulPropertyContainer, vr::Prop_UserHeadToEyeDepthMeters_Float, 0.16f);
	vr::VRProperties()->SetFloatProperty(m_ulPropertyContainer, vr::Prop_DisplayFrequency_Float, this->DisplayFrequency);
	vr::VRProperties()->SetFloatProperty(m_ulPropertyContainer, vr::Prop_SecondsFromVsyncToPhotons_Float, this->SecondsFromVsyncToPhotons);

	// avoid "not fullscreen" warnings from vrmonitor
	vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_IsOnDesktop_Bool, false);
}
