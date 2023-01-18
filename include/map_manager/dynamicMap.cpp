/*
	FILE: dynamicMap.cpp
	--------------------------------------
	function definition of dynamic map
*/

#include <ros/ros.h>
#include <map_manager/dynamicMap.h>

namespace mapManager{
	dynamicMap::dynamicMap(){
		this->ns_ = "dynamic_map";
		this->hint_ = "[dynamicMap]";
		this->detector_.reset(new mapManager::dynamicDetector (this->nh_));
	}

	dynamicMap::dynamicMap(const ros::NodeHandle& nh){
		this->ns_ = "dynamic_map";
		this->hint_ = "[dynamicMap]";
		this->detector_.reset(new mapManager::dynamicDetector (this->nh_));
		this->initMap(nh);
	}


	void dynamicMap::getDynamicObstacles(std::vector<Eigen::Vector3d>& obstaclePos,
										 std::vector<Eigen::Vector3d>& obstacleVel,
										 std::vector<Eigen::Vector3d>& obstacleSize){

	}
}

