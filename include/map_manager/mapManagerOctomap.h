/*
	FILE: mapManagerOctomap.h
	--------------------------
	Used as a rosservice to perform collision checking
*/
#ifndef MAPMANAGEROCTOMAP
#define MAPMANAGEROCTOMAP

#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/conversions.h>
#include <map_manager/CheckCollision.h>

using std::cout; using std::endl;

namespace mapManager{
	class mapManagerOctomap{
	private:
		ros::NodeHandle nh_;
		octomap::OcTree* map_;
		ros::ServiceClient mapClient_;
		ros::ServiceServer collisionServer_;

	public:
		mapManagerOctomap(const ros::NodeHandle& nh);
		void updateMap();

		// collision checking service
		bool checkCollision(map_manager::CheckCollision::Request &req, map_manager::CheckCollision::Response &res);
	};
}

#endif