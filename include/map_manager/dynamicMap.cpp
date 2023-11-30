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
	}

	dynamicMap::dynamicMap(const ros::NodeHandle& nh, bool freeMap){
		this->ns_ = "dynamic_map";
		this->hint_ = "[dynamicMap]";
		this->initMap(nh, freeMap);
	}

	void dynamicMap::initMap(const ros::NodeHandle& nh, bool freeMap){
		this->nh_ = nh;
		this->initParam();
		this->initPrebuiltMap();
		this->registerPub();
		this->registerCallback();
		this->detector_.reset(new mapManager::dynamicDetector (this->nh_));
		if (freeMap){
        	this->freeMapTimer_ = this->nh_.createTimer(ros::Duration(0.01), &dynamicMap::freeMapCB, this);
		}
	}

	void dynamicMap::freeMapCB(const ros::TimerEvent&){
		std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> freeRegions;
		std::vector<mapManager::box3D> dynamicBBoxes;
		this->detector_->getDynamicObstacles(dynamicBBoxes);
		for (mapManager::box3D ob:dynamicBBoxes){
			Eigen::Vector3d lowerBound (ob.x-ob.x_width/2-0.3, ob.y-ob.y_width/2-2*this->mapRes_-0.3, 0.0);
			Eigen::Vector3d upperBound (ob.x+ob.x_width/2+0.3, ob.y+ob.y_width/2+2*this->mapRes_+0.3, ob.z+ob.z_width+2*this->mapRes_+this->robotSize_(2)/2);
			freeRegions.push_back(std::make_pair(lowerBound, upperBound));
		}
		this->freeRegions(freeRegions);
		this->updateFreeRegions(freeRegions);
	}

	void dynamicMap::getDynamicObstacles(std::vector<Eigen::Vector3d>& obstaclePos,
										 std::vector<Eigen::Vector3d>& obstacleVel,
										 std::vector<Eigen::Vector3d>& obstacleSize){
		std::vector<mapManager::box3D> dynamicBBoxes;
		this->detector_->getDynamicObstacles(dynamicBBoxes);
		for (size_t i=0 ; i<dynamicBBoxes.size() ; ++i){
			Eigen::Vector3d pos(dynamicBBoxes[i].x, dynamicBBoxes[i].y, dynamicBBoxes[i].z);
			Eigen::Vector3d vel(dynamicBBoxes[i].Vx, dynamicBBoxes[i].Vy, 0);
			Eigen::Vector3d size(dynamicBBoxes[i].x_width, dynamicBBoxes[i].y_width, dynamicBBoxes[i].z_width);
			obstaclePos.push_back(pos);
			obstacleVel.push_back(vel);
			obstacleSize.push_back(size);
		}
	}

}

