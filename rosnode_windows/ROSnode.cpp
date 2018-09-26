#include <iostream>
#include <list>
#include "ros.h"
#include "FileMapping.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Char.h"
#include "Joint.h"
#include <list>

//#define RES_WID 640
//#define RES_HEI 480
// RES_WID * RES_HEI * 16 // 4 bytes each for {X, Y, Z, Col}

/* Memory layout
 *
 * Offset	Description			Notes
 *  0		 Number of joints	 Updated as more are added
 *
 *  1		 Joint position		 4 bytes long (from float)
 *  5		 Joint name length	 Number of characters
 *  6		 Joint name			 As long as joint name length
 *
 *  after	 Joint position		 Repeat above block
 *			 Joint name length	 etc.
 *			 Joint name			 etc.
 */

// Create file mapping, needs to be global
FileMapping mapJoints("ROSjoints", 1000); // Expecting ~250
FileMapping mapHands_l("ROShands_l", 50); // Expecting 32 (4*7 + 4)
FileMapping mapHands_r("ROShands_r", 50); // Expecting 32 (4*7 + 4)
FileMapping mapCloud("ROScloud", 65000); // Expecting up to 65536ish

// IP Address of Baxter computer
// char *ip = "hostname.server.com:11411";
char *ip = "192.168.0.100";

std::vector<Joint> joints; // Local joints list
geometry_msgs::Pose pose_msg_l;
geometry_msgs::Pose pose_msg_r;
std_msgs::Char grab_msg_l; // 'g' or 'u' for grab and ungrab
std_msgs::Char grab_msg_r; // 'g' or 'u' for grab and ungrab

// Subscribers are further down
ros::Publisher hand_position_publisher_l("/vr/pose_l", &pose_msg_l);
ros::Publisher hand_position_publisher_r("/vr/pose_r", &pose_msg_r);
ros::Publisher hand_grab_publisher_l("/vr/grab_l", &grab_msg_l);
ros::Publisher hand_grab_publisher_r("/vr/grab_r", &grab_msg_r);

bool grabbing_l;
bool grabbing_r;

// Writes each point at a time, sequentially
void point_cloud_callback(const sensor_msgs::PointCloud2& msg) {
	std::cerr << "C";

	int offset(0);
	// Messy, but writes the length first. Float because int8 is too small
	mapCloud.write_data((float)msg.data_length/16, offset);
	offset += sizeof(float);
	for (int i(0); i < msg.data_length; i++) {
		mapCloud.write_data(msg.data[i], offset);
		offset++;
	}
	//std::cout << "msg:" << std::endl;
	//for (int i(0); i < 32; i++) {
		//std::cout << "  " << i << ": " << (int)msg.data[i] <<std::endl;
	//}
	//std::cout << std::endl;
	mapCloud.commit_data(); // Actually write to NSM
}

void joint_states_callback(const sensor_msgs::JointState& msg) {
	//std::cerr << "J";
	bool exists = false;
	for (int i(0); i < msg.position_length; i++) {
		std::vector<Joint>::iterator iterator;
		for (iterator = joints.begin(); iterator != joints.end(); ++iterator) {
			// We have already heard about this joint, move on
			if (iterator->exists(msg.name[i])) {
				exists = true;
				break;
			}
		}
		if (!exists) {
			// Add new joint to list
			std::cerr << "Found: " << msg.name[i] << std::endl;
			int offset(1);
			if (joints.size() > 0)
				offset += joints.back().get_offset() + joints.back().get_size();
			Joint tempJoint(msg.name[i], offset);
			joints.push_back(tempJoint);
			// Update joints count
			mapJoints.write_data((uint8_t)joints.size(), 0);
			// Write data, then joint name length, then joint name
			mapJoints.write_data((float)msg.position[i], offset);
			offset += sizeof(float);
			mapJoints.write_data((uint8_t)strlen(msg.name[i]), offset);
			offset++;
			mapJoints.write_data(msg.name[i], offset);
			offset += strlen(msg.name[i]);
		} else {
			// It's already in the list, at iterator
			mapJoints.write_data((float)msg.position[i], iterator->get_offset());
		}
	}
	mapJoints.commit_data(); // Actually write to NSM
}

// Gets gripper status
//void gripper_l_callback(const baxter_core_msgs::EndEffectorState& msg) {
	// I didn't implement it though
	//msg.data.position, 0-100
//}

// Writes left IK solver status to mapHands
void ik_status_l_callback(const std_msgs::Char& msg) {
	// Writes status byte in second byte (from left offset)
	mapHands_l.write_data((uint8_t)msg.data, 1);
	mapHands_l.commit_data(); // Actually write to NSM
	if (msg.data == 'f')
		std::cout << "IK_l fail for last point" << std::endl;
	else if (msg.data == 'c')
		std::cout << "Move_l completed" << std::endl;
	else
		std::cout << "Got a '" << msg.data << "' unexpectedly from ik_status_l" << std::endl;
}

// Writes right IK solver status to mapHands
void ik_status_r_callback(const std_msgs::Char& msg) {
	// Writes status byte in second byte (from left offset)
	mapHands_r.write_data((uint8_t)msg.data, (8*sizeof(float))+1);
	mapHands_r.commit_data(); // Actually write to NSM
	if (msg.data == 'f')
		std::cout << "IK_r fail for last point" << std::endl;
	else if (msg.data == 'c')
		std::cout << "Move_r completed" << std::endl;
	else
		std::cout << "Got a '" << msg.data << "' unexpectedly from ik_status_r" << std::endl;
}

// Left hand
void get_pose_l() {
	geometry_msgs::Point point;
	geometry_msgs::Quaternion quat;

	// Check for changes
	int offset(0);
	if (mapHands_l.read_data(offset) != (uint8_t)1)
		return;
	else {
		// Reset the 'changed' status byte
		mapHands_l.write_data((uint8_t)offset, 0);
		mapHands_l.commit_data(4);
	}
	offset += sizeof(float);

	// Set message data
	double* data[] = {
		&point.x, &point.y, &point.z,
		&quat.x, &quat.y, &quat.z, &quat.w
	};
	for (int i(0); i < sizeof(data)/sizeof(double); i++) {
		*data[i] = (double)mapHands_l.read_data_float(offset);
		offset += sizeof(float);
	}
	pose_msg_l.position = point;
	pose_msg_l.orientation = quat;
	hand_position_publisher_l.publish(&pose_msg_l);

	std::cerr << "L:" << std::endl;
	std::cerr << "  " << (float)pose_msg_l.position.x << std::endl;
	std::cerr << "  " << (float)pose_msg_l.position.y << std::endl;
	std::cerr << "  " << (float)pose_msg_l.position.z << std::endl;
	std::cerr << "  " << (float)pose_msg_l.orientation.x << std::endl;
	std::cerr << "  " << (float)pose_msg_l.orientation.y << std::endl;
	std::cerr << "  " << (float)pose_msg_l.orientation.z << std::endl;
	std::cerr << "  " << (float)pose_msg_l.orientation.w << std::endl << std::endl;
}

// Right hand
void get_pose_r() {
	geometry_msgs::Point point;
	geometry_msgs::Quaternion quat;

	// Check for changes
	int offset(0);
	if (mapHands_r.read_data(offset) != (uint8_t)1)
		return;
	else {
		// Reset the 'changed' status byte
		mapHands_r.write_data((uint8_t)offset, 0);
		mapHands_r.commit_data(4);
	}
	offset += sizeof(float);

	// Set message data
	double* data[] = {
		&point.x, &point.y, &point.z,
		&quat.x, &quat.y, &quat.z, &quat.w
	};
	for (int i(0); i < sizeof(data)/sizeof(double); i++) {
		*data[i] = (double)mapHands_r.read_data_float(offset);
		offset += sizeof(float);
	}
	pose_msg_r.position = point;
	pose_msg_r.orientation = quat;
	hand_position_publisher_r.publish(&pose_msg_r);

	std::cerr << "R:" << std::endl;
	std::cerr << "  " << (float)pose_msg_r.position.x << std::endl;
	std::cerr << "  " << (float)pose_msg_r.position.y << std::endl;
	std::cerr << "  " << (float)pose_msg_r.position.z << std::endl;
	std::cerr << "  " << (float)pose_msg_r.orientation.x << std::endl;
	std::cerr << "  " << (float)pose_msg_r.orientation.y << std::endl;
	std::cerr << "  " << (float)pose_msg_r.orientation.z << std::endl;
	std::cerr << "  " << (float)pose_msg_r.orientation.w << std::endl << std::endl;
}

// Left hand
void get_grab_l() {
	char data = (char)mapHands_l.read_data(8 * sizeof(float)); // Offset of grab status
	if (((data == 'g') && !grabbing_l) || ((data == 'u') && grabbing_l)) {
		grab_msg_l.data = data;
		hand_grab_publisher_l.publish(&grab_msg_l);
		grabbing_l = !grabbing_l;
		std::cerr << std::endl << "LEFT!" << std::endl;
		mapHands_l.write_data((uint8_t)'n', 8*sizeof(float));
		mapHands_l.commit_data();
	}
}

// Right hand
void get_grab_r() {
	char data = (char)mapHands_r.read_data(8 * sizeof(float)); // Offset of grab status
	if (((data == 'g') && !grabbing_r) || ((data == 'u') && grabbing_r)) {
		grab_msg_r.data = data;
		hand_grab_publisher_r.publish(&grab_msg_r);
		grabbing_r = !grabbing_r;
		std::cerr << std::endl << "RIGHT!" << std::endl;
		mapHands_r.write_data((uint8_t)'n', 8*sizeof(float));
		mapHands_r.commit_data();
	}
}


int main(int argc, char* argv[]) {
	ros::NodeHandle nodeHandle;
	// Open file mapping
	int fail = mapJoints.open_fileMap() || mapHands_l.open_fileMap() || mapHands_r.open_fileMap() || mapCloud.open_fileMap();
	if (fail != 0)
		return EXIT_FAILURE;

	// Connect to rosserial server
	std::cout << "Connecting to server at " << ip << std::endl;
	nodeHandle.initNode(ip);
	nodeHandle.advertise(hand_position_publisher_l);
	nodeHandle.advertise(hand_position_publisher_r);
	nodeHandle.advertise(hand_grab_publisher_l);
	nodeHandle.advertise(hand_grab_publisher_r);

	// Publish/Subscribe to topics
	// Publishers are globals further up
	ros::Subscriber<sensor_msgs::JointState> joint_states_subscriber("/robot/joint_states", &joint_states_callback);
	ros::Subscriber<sensor_msgs::PointCloud2> point_cloud_subscriber("/vr/cloud", &point_cloud_callback); //camera/depth_registered/points
	ros::Subscriber<std_msgs::Char> ik_status_l_subscriber("/vr/ik_status_l", &ik_status_l_callback);
	ros::Subscriber<std_msgs::Char> ik_status_r_subscriber("/vr/ik_status_r", &ik_status_r_callback);
	nodeHandle.subscribe(joint_states_subscriber);
	nodeHandle.subscribe(point_cloud_subscriber);
	nodeHandle.subscribe(ik_status_l_subscriber);
	nodeHandle.subscribe(ik_status_r_subscriber);

	// Initially set the length status to zero
	mapCloud.write_data(0.0f, 0);
	mapCloud.commit_data();
	// Assume Baxter's grippers are open
	grabbing_l = false;
	grabbing_r = false;

	std::cout << "Checking for new messages" << std::endl;
	while (true) {
		nodeHandle.spinOnce();
		get_pose_l();
		get_pose_r();
		get_grab_l();
		get_grab_r();
		//mapHands.debug_memory();
		Sleep(20);
	};

	std::cout << "Exited naturally" << std::endl;
	return 0;
}
