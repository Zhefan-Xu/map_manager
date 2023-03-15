#include <ros/ros.h>
#include <map_manager/dynamicMapUGV.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "dyanmic_map_ugv_node");
	ros::NodeHandle nh;

	mapManager::dynamicMapUGV m(nh);

	ros::spin();

	return 0;
}