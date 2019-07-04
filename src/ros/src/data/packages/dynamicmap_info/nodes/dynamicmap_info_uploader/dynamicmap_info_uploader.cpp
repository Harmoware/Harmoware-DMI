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
#include <cstdio>
#include <time.h>
#include <pthread.h>
#include <tuple>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <dynamicmap_info.h>
#include <is/DmManager.h>

#define	IS_DEBUG	(1)
#define DEBUG_PRINT	(0)

#define MYNAME		"dynamicmap_info_uploader"
#define OWN_TOPIC_NAME	"current_pose"
#define CAR_TOPIC_NAME	"obj_car/obj_pose"
#define PERSON_TOPIC_NAME	"obj_person/obj_pose"
#define OWN_TOPIC_NUM	1	
#define CAR_TOPIC_NUM	2
#define PERSON_TOPIC_NUM	3

using namespace std;

//store subscribed value
static std::vector <jsk_recognition_msgs::BoundingBoxArray> car_positions_array;
static std::vector <jsk_recognition_msgs::BoundingBoxArray> person_positions_array;
//store own position and direction now.updated by position_getter
static std::vector <geometry_msgs::PoseStamped> current_pose_position;

static int sleep_msec = 250;		// period

pthread_mutex_t pose_lock_;

static DatagramSocket* sendSock;
static char mac_addr[MAC_ADDRBUFSIZ];
static string ip;
static int port;

static void makeSendTuple(const std::vector <jsk_recognition_msgs::BoundingBoxArray>& positions_array, const int type, vector<Tuple>& tuples)
{
#if DEBUG_PRINT
	cout << "*** type=" << type << ": posisions_array.size=" << positions_array.size() << " ***" << endl;
#endif

	for(auto position: positions_array) {
#if DEBUG_PRINT
		if(position.boxes.size() > 1) cout << " @@ type=" << type << ": boxes size=" << position.boxes.size() << " @@" << endl;
#endif
		for (jsk_recognition_msgs::BoundingBox tmpBox : position.boxes) {
			long now = DmUtil::getTimeMillisec();
			Tuple tuple(24);
			tuple.setValue(0, (string)mac_addr, now);
			tuple.setValue(1, type, now);
			tuple.setValue(2, (long)position.header.seq, now);
			tuple.setValue(3, (long)position.header.stamp.sec, now);
			tuple.setValue(4, (long)position.header.stamp.nsec, now);
			tuple.setValue(5, position.header.frame_id, now);
			
			tuple.setValue(6, (int)position.boxes.size(), now);
			tuple.setValue(7, (long)tmpBox.header.seq, now);
			tuple.setValue(8, (long)tmpBox.header.stamp.sec, now);
			tuple.setValue(9, (long)tmpBox.header.stamp.nsec, now);
			tuple.setValue(10, tmpBox.header.frame_id, now);

			tuple.setValue(11, (double)tmpBox.pose.position.x, now);
			tuple.setValue(12, (double)tmpBox.pose.position.y, now);
			tuple.setValue(13, (double)tmpBox.pose.position.z, now);

			tuple.setValue(14, (double)tmpBox.pose.orientation.x, now);
			tuple.setValue(15, (double)tmpBox.pose.orientation.y, now);
			tuple.setValue(16, (double)tmpBox.pose.orientation.z, now);
			tuple.setValue(17, (double)tmpBox.pose.orientation.w, now);

			tuple.setValue(18, (double)tmpBox.dimensions.x, now);
			tuple.setValue(19, (double)tmpBox.dimensions.y, now);
			tuple.setValue(20, (double)tmpBox.dimensions.z, now);

			tuple.setValue(21, (double)tmpBox.value, now);
			tuple.setValue(22, (int)tmpBox.label, now);

			tuple.setValue(23, DmUtil::getTimeMillisecStr(), now);

			tuples.push_back(tuple);
#ifdef IS_DEBUG
			try {
				sendSock->sendStreamData("dynamic_map_info", tuples);
			}
			catch (const ConnectionFailedException& e) {
				cout << "failed sendStreamData()" << endl;
			}
			tuples.clear();
#endif
		}
	}



	return;
}

static void makeSendTuplePose(const std::vector <geometry_msgs::PoseStamped>& position, vector<Tuple>& tuples)
{
	long now = DmUtil::getTimeMillisec();
	Tuple tuple(24);
#if DEBUG_PRINT
	cout << "+++ type=1 position_size: " << position.size() << " +++" << endl;
#endif
	for(size_t i = 0; i < position.size(); i++) {
		tuple.setValue(0, (string)mac_addr, now);
		tuple.setValue(1, OWN_TOPIC_NUM, now);
		tuple.setValue(2, (long)position[i].header.seq, now);
		tuple.setValue(3, (long)position[i].header.stamp.sec, now);
		tuple.setValue(4, (long)position[i].header.stamp.nsec, now);
		tuple.setValue(5, position[i].header.frame_id, now);
			
		tuple.setValue(6, 1, now);
		tuple.setValue(7, 0, now);
		tuple.setValue(8, 0, now);
		tuple.setValue(9, 0, now);
		tuple.setValue(10, position[i].header.frame_id, now);
		
		tuple.setValue(11, (double)position[i].pose.position.x, now);
		tuple.setValue(12, (double)position[i].pose.position.y, now);
		tuple.setValue(13, (double)position[i].pose.position.z, now);

		tuple.setValue(14, (double)position[i].pose.orientation.x, now);
		tuple.setValue(15, (double)position[i].pose.orientation.y, now);
		tuple.setValue(16, (double)position[i].pose.orientation.z, now);
		tuple.setValue(17, (double)position[i].pose.orientation.w, now);

		tuple.setValue(18, (double)0, now);
		tuple.setValue(19, (double)0, now);
		tuple.setValue(20, (double)0, now);
		tuple.setValue(21, (double)0, now);
		tuple.setValue(22, (int)0, now);

		tuple.setValue(23, DmUtil::getTimeMillisecStr(), now);

		tuples.push_back(tuple);
#ifdef IS_DEBUG
		try {
			sendSock->sendStreamData("dynamic_map_info", tuples);
		}
		catch (const ConnectionFailedException& e) {
			cout << "failed sendStreamData()" << endl;
		}
		tuples.clear();
#endif
	}

	return;
}

static void send_streamdata()
{
	vector<Tuple> tuples;
	//get data of car and person recognizing
	pthread_mutex_lock(&pose_lock_);
	if(current_pose_position.size() > 0) makeSendTuplePose(current_pose_position, tuples);
	if(car_positions_array.size() > 0) makeSendTuple(car_positions_array, CAR_TOPIC_NUM, tuples);
	if(person_positions_array.size() > 0) makeSendTuple(person_positions_array, PERSON_TOPIC_NUM, tuples);
	
#ifndef IS_DEBUG
	sendSock->sendStreamData("dynamic_map_info", tuples);
#endif

	current_pose_position.clear();
	car_positions_array.clear();
	car_positions_array.shrink_to_fit();
	person_positions_array.clear();
	person_positions_array.shrink_to_fit();
	tuples.clear();
	tuples.shrink_to_fit();
	pthread_mutex_unlock(&pose_lock_);

	return;
}

static void* intervalCall(void *unused)
{
	while(1){
		//If angle and position data is not updated from previous data send,
		//data is not sent
		if((car_positions_array.size() + person_positions_array.size() + current_pose_position.size()) <= 0) {
			usleep(sleep_msec*1000);
			continue;
		}

		send_streamdata();
		usleep(sleep_msec*1000);
	}

	return nullptr;
}

static void car_locate_cb(const jsk_recognition_msgs::BoundingBoxArray& obj_pose_msg)
{
	if (obj_pose_msg.boxes.size() > 0) {
		pthread_mutex_lock(&pose_lock_);
		car_positions_array.push_back(obj_pose_msg);
		pthread_mutex_unlock(&pose_lock_);
	}
}

static void person_locate_cb(const jsk_recognition_msgs::BoundingBoxArray &obj_pose_msg)
{
	if (obj_pose_msg.boxes.size() > 0) {
		pthread_mutex_lock(&pose_lock_);
		person_positions_array.push_back(obj_pose_msg);
		pthread_mutex_unlock(&pose_lock_);
	}
}

static void current_pose_cb(const geometry_msgs::PoseStamped &pose)
{
	pthread_mutex_lock(&pose_lock_);
	current_pose_position.push_back(pose);
	pthread_mutex_unlock(&pose_lock_);
}

int main(int argc, char **argv)
{
	ros::init(argc ,argv, MYNAME);
	std::cout << MYNAME << std::endl;

	pose_lock_ = PTHREAD_MUTEX_INITIALIZER;

	probe_mac_addr(mac_addr);

	ros::NodeHandle nh;
	ros::Subscriber car_locate = nh.subscribe("/" CAR_TOPIC_NAME, 1, car_locate_cb);
	ros::Subscriber person_locate = nh.subscribe("/" PERSON_TOPIC_NAME, 1, person_locate_cb);
	ros::Subscriber gnss_pose = nh.subscribe("/" OWN_TOPIC_NAME, 1, current_pose_cb);

	nh.param<string>("/dynamicmap_info_uploader/ip_addr", ip, "127.0.0.1");
	nh.param<int>("/dynamicmap_info_uploader/port", port, 9002);

	pthread_t th;
	if(pthread_create(&th, nullptr, intervalCall, nullptr)){
		printf("thread create error\n");
	}
	pthread_detach(th);

	// Retrieve UDP sending object
	try {
		sendSock = DmManager::getDatagramSocket(ip, port);
	}
	catch (const ConnectionFailedException& e) {
		cout << "Could not connect to dynamic prototype DB system" << endl;
		cout << "Exit " << MYNAME << endl;
		return -1;
	}
  

	ros::spin();
	return 0;
}
