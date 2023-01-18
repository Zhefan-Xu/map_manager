#include <ros/ros.h>
#include <map_manager/detector/dynamicDetector.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "dyanmic_detector_node");
	ros::NodeHandle nh;

	mapManager::dynamicDetector d (nh);

	ros::spin();

	return 0;
}