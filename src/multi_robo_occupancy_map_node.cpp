#include <ros/ros.h>
#include <map_manager/multiRoboOccupancyMap.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "multi_robot_occcupancy_map_node");
	ros::NodeHandle nh;

	mapManager::multiRoboOccMap m;
	m.initMultiRoboOccMap(nh);

	ros::spin();

	return 0;
}