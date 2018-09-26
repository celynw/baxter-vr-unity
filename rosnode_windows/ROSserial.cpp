#include "ROSserial.h"
#include <iostream>

ROSserial::ROSserial(char *newIP, FileMapping *newFileMap) {
	ip = newIP; // Set IP address of host
	fileMap = newFileMap; // Sets this up with pre-existing object
	std::cerr << "Connecting to server at " << ip << std::endl;
	nodeHandle.initNode(ip); // Initialise
}

// Writes each point at a time, sequentially
void ROSserial::message_callback(const sensor_msgs::PointCloud2& msg) {
	const int messageLength = 32; // Points are 32 bytes long
	std::vector<uint8_t> buffer;
	int iteration(0);
	for (int i = 0; i < msg.data_length; i++) {
		iteration = i % messageLength;
		buffer.push_back(msg.data[i]);
		// Write point
		///if (iteration == messageLength-1)
			///fileMap->write_data(buffer, OFFSET);
	}
}

// Create subscriber in main thread, then use this
void ROSserial::subscribe(ros::Subscriber<sensor_msgs::PointCloud2> *subscriber) {
	nodeHandle.subscribe(*subscriber);
}

// spinOnce()
void ROSserial::update() {
	nodeHandle.spinOnce();
}
