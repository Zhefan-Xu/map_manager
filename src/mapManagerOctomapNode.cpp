/*
	FILE: mapManagerOctomapNode.cpp
	-------------------------------
	map manager node for octomap
*/

#include<ros/ros.h>
#include<map_manager/mapManagerOctomap.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "map_manager_octomap");
	ros::NodeHandle nh;
	mapManager::mapManagerOctomap m (nh);

	return 0;
}