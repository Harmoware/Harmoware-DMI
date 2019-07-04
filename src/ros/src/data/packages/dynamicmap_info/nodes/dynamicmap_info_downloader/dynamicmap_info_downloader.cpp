/*
 * Copyright 2019 Nagoya University
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <is/DmManager.h>

#include <cstdio>
#include <string>
#include <stdint.h>
#include <time.h>
#include <pthread.h>
#include <tuple>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <dynamicmap_info.h>

#define DEBUG_PRINT	(0)

#define MYNAME		"dynamicmap_info_downloader"


#define TYPE_OWN	(1)
#define TYPE_CAR	(2)
#define TYPE_PEDESTRIAN	(3)

using namespace std;

static ros::Publisher current_pose_pub;
static ros::Publisher car_obj_pose_pub;
static ros::Publisher person_obj_pose_pub;

static Connection* con;

static char mac_addr[MAC_ADDRBUFSIZ];
static string ip;
static int port;
static int range_msec;
static int ignore_my_pose = 1;
static	jsk_recognition_msgs::BoundingBoxArray pub_msg_;

static int cnt_own = 0;
static int cnt_car = 0;
static int cnt_person = 0;

void callbackFunc(ResultSet rs)
{
        while(rs.next()) {
		ResultSetMetaData rsmd = rs.getResultSetMetaData();
		int type = rs.getInt("type");
		if(type == TYPE_OWN) {
			geometry_msgs::PoseStamped current_pose_msg;
			current_pose_msg.header.seq = (uint32_t)rs.getLong("header_seq");
			current_pose_msg.header.stamp.sec = (uint32_t)rs.getLong("header_secs");
			current_pose_msg.header.stamp.nsec = (uint32_t)rs.getLong("header_nsecs");
			current_pose_msg.header.frame_id = rs.getString("frame_id");
			current_pose_msg.pose.position.x = rs.getDouble("x");
			current_pose_msg.pose.position.y = rs.getDouble("y");
			current_pose_msg.pose.position.z = rs.getDouble("z");
			current_pose_msg.pose.orientation.x = rs.getDouble("or_x");
			current_pose_msg.pose.orientation.y = rs.getDouble("or_y");
			current_pose_msg.pose.orientation.z = rs.getDouble("or_z");
			current_pose_msg.pose.orientation.w = rs.getDouble("or_w");

			current_pose_pub.publish(current_pose_msg);
			cnt_own++;
		}

		if(type == TYPE_CAR || type == TYPE_PEDESTRIAN) {
			jsk_recognition_msgs::BoundingBoxArray pub_msg;
			jsk_recognition_msgs::BoundingBox tmpbox;
			pub_msg.header.seq = (uint32_t)rs.getLong("header_seq");
			pub_msg.header.stamp.sec = (uint32_t)rs.getLong("header_secs");
			pub_msg.header.stamp.nsec = (uint32_t)rs.getLong("header_nsecs");
			pub_msg.header.frame_id = rs.getString("frame_id");

			tmpbox.header.seq = (uint32_t)rs.getLong("boxes_seq");
			tmpbox.header.stamp.sec = (uint32_t)rs.getLong("boxes_secs");
			tmpbox.header.stamp.nsec = (uint32_t)rs.getLong("boxes_nsecs");
			tmpbox.header.frame_id = rs.getString("boxes_frame_id");
			tmpbox.pose.position.x = rs.getDouble("x");
			tmpbox.pose.position.y = rs.getDouble("y");
			tmpbox.pose.position.z = rs.getDouble("z");
			tmpbox.pose.orientation.x = rs.getDouble("or_x");
			tmpbox.pose.orientation.y = rs.getDouble("or_y");
			tmpbox.pose.orientation.z = rs.getDouble("or_z");
			tmpbox.pose.orientation.w = rs.getDouble("or_w");
			tmpbox.dimensions.x = rs.getDouble("dx");
			tmpbox.dimensions.y = rs.getDouble("dy");
			tmpbox.dimensions.z = rs.getDouble("dz");
			tmpbox.value = rs.getDouble("value");
			tmpbox.label = rs.getInt("label");

			pub_msg.boxes.push_back(tmpbox);


			if(type == TYPE_CAR) {car_obj_pose_pub.publish(pub_msg); cnt_car++;}
			if(type == TYPE_PEDESTRIAN) {person_obj_pose_pub.publish(pub_msg); cnt_person++;}
		}
	}
}

int main(int argc, char **argv)
{
	ros::init(argc ,argv, MYNAME);
	ros::NodeHandle nh;

	std::cout << MYNAME << std::endl;

	if(argc > 1) {
		if(strncmp(argv[1], "show_my_pose", 12) == 0) ignore_my_pose = 0;
	}
	std::cerr << "ignore_my_pose=" << ignore_my_pose << std::endl;

	current_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/dynamicmap_info/current_pose", 1, false);
	car_obj_pose_pub = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("/dynamicmap_info/car_pose", 1, false);
	person_obj_pose_pub =  nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("/dynamicmap_info/person_pose", 1, false);

	nh.param<string>("/dynamicmap_info_downloader/ip_addr", ip, "127.0.0.1");
	nh.param<int>("/dynamicmap_info_downloader/port", port, 9001);
	nh.param<int>("/dynamicmap_info_downloader/query_interval_sec", range_msec, 1000);

	probe_mac_addr(mac_addr);

	string query = "master dynamic_map_info select * from dynamic_map_info [range " + std::to_string(range_msec) + " msec]";

	// Connection dynamicmap_prototype server
	try {
		con = DmManager::getDBConnection(ip, port);
	}
	catch (const ConnectionFailedException& e) {
		cout << "Could not connect to dynamic prototype DB system" << endl;
		cout << "Exit " << MYNAME << endl;
		return -1;
	}

	unsigned int mngId = 0;
	// Register continuous query on server
	try {
		mngId = con->registerQuery(query, (FUNC_CALLBACK)callbackFunc);
	}
	catch (const ConnectionTimeoutException& e) {
		cout << "Connection Timeout error" << endl;
	}
	catch (const SQLException& e) {
		cout << "Query syntax error" <<  endl;
	}

	ros::spin();

	// Disconnecting from server
	try {
	        con->reconnect();
		con->cancelQuery(mngId);
		con->disconnect();
	}
	catch (...) {
	}

	cout << "cnt_own=" << cnt_own << ", cnt_car=" << cnt_car << ", cnt_person=" << cnt_person << endl;

	return 0;
}
