/*
	FILE: dynamicMap.cpp
	--------------------------------------
	function definition of dynamic map
*/

#include <ros/ros.h>
#include <map_manager/dynamicMap.h>

namespace mapManager{
	dynamicMap::dynamicMap(const ros::NodeHandle& nh){
		this->ns_ = "dynamic_map";
		this->hint_ = "[dynamicMap]";
		this->detector_.reset(new mapManager::dynamicDetector (this->nh_));
		this->initMap(nh);
		// tracking timer
        this->freeMapTimer_ = this->nh_.createTimer(ros::Duration(0.01), &dynamicMap::freeMapCB, this);
	}

	void dynamicMap::freeMapCB(const ros::TimerEvent&){
		std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> freeRegions;
		std::vector<mapManager::box3D> dynamicBBoxes;
		this->detector_->getDynamicObstacles(dynamicBBoxes);
		for (mapManager::box3D ob:dynamicBBoxes){
			Eigen::Vector3d lowerBound (ob.x-ob.x_width/2-0.3, ob.y-ob.y_width/2-0.3, this->robotSize_(2));
			Eigen::Vector3d upperBound (ob.x+ob.x_width/2+0.3, ob.y+ob.y_width/2+0.3, ob.z+ob.z_width+0.2);
			freeRegions.push_back(std::make_pair(lowerBound, upperBound));
		}
		this->updateFreeRegions(freeRegions);
	}

	void dynamicMap::getDynamicObstacles(std::vector<Eigen::Vector3d>& obstaclePos,
										 std::vector<Eigen::Vector3d>& obstacleVel,
										 std::vector<Eigen::Vector3d>& obstacleSize){

	}

}

