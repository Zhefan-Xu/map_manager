/*
	FILE: mapManagerOctomap.cpp
	---------------------------
	Function definition of octomap manager
*/
#include<map_manager/mapManagerOctomap.h>

namespace mapManager{
	mapManagerOctomap::mapManagerOctomap(const ros::NodeHandle& nh) : nh_(nh){
		this->mapClient_ = this->nh_.serviceClient<octomap_msgs::GetOctomap>("/octomap_binary");
		this->updateMap();

		// start collision checking server:
		this->collisionServer_ = this->nh_.advertiseService("check_collision_octomap", &mapManagerOctomap::checkCollision, this);
		ros::spin();
	}

	void mapManagerOctomap::updateMap(){
		octomap_msgs::GetOctomap mapSrv;
		bool serviceSuccess = this->mapClient_.call(mapSrv);
		ros::Rate rate(10);
		while (not serviceSuccess and ros::ok()){
			serviceSuccess = this->mapClient_.call(mapSrv);
			ROS_INFO("[Map Manager Octomap]: Wait for octomap service...");
			rate.sleep();
		}
		octomap::AbstractOcTree* abtree = octomap_msgs::binaryMsgToMap(mapSrv.response.map);
		this->map_ = dynamic_cast<octomap::OcTree*>(abtree);
		cout << "[Map Manager Octomap]: Map updated!" << endl;
	}

	bool mapManagerOctomap::checkCollision(map_manager::CheckCollision::Request &req, map_manager::CheckCollision::Response &res){
		return true;
	}
}