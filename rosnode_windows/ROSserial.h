//#pragma once
#ifndef ROSSERIAL_H_
#define ROSSERIAL_H_

#include "ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "fileMapping.h"

class ROSserial {
	private:
		ros::NodeHandle nodeHandle;
		FileMapping *fileMap; // For writing the data to
	public:
		char *ip; // IP address of host
		ROSserial() = delete; // Don't use default constructor
		ROSserial(char *ip, FileMapping *newFileMap);
		void message_callback(const sensor_msgs::PointCloud2& msg);
		//void(*cbPtr)(const sensor_msgs::PointCloud2&);
		void subscribe(ros::Subscriber<sensor_msgs::PointCloud2> *subscriber);
		void update(); // spinOnce()
};

#endif // ROSSERIAL_H_
