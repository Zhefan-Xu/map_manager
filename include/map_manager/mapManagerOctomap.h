/*
	FILE: mapManagerOctomap.h
	--------------------------
	Used as a rosservice to perform collision checking
*/
#ifndef MAPMANAGEROCTOMAP
#define MAPMANAGEROCTOMAP

#include <ros/ros.h>
#include <random>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/conversions.h>
#include <map_manager/CheckCollision.h>
#include <map_manager/RandomSample.h>

using std::cout; using std::endl;

namespace mapManager{
	#define PI_const 3.1415926
	std::random_device rd;
	std::mt19937 mt(rd());

	double randomNumber(double min, double max){
		std::uniform_real_distribution<double> distribution(min, max);
		return distribution(mt);
	}

	class mapManagerOctomap{
	private:
		ros::NodeHandle nh_;
		octomap::OcTree* map_;
		double mapRes_;
		ros::ServiceClient mapClient_;
		ros::ServiceServer collisionServer_;
		ros::ServiceServer sampleServer_;

		double sampleRegion_[6];


	public:
		mapManagerOctomap(const ros::NodeHandle& nh);
		void updateMap();
		void updateSampleRegion();

		// collision checking service
		bool checkCollisionPoint(const octomap::point3d &p, bool ignoreUnknown=false);
		bool checkCollision(const octomap::point3d &p, double xsize, double ysize, double zsize, bool ignoreUnknown=false);
		bool checkCollision(map_manager::CheckCollision::Request &req, map_manager::CheckCollision::Response &res);
	
		// random sample service
		octomap::point3d randomSample(double xsize, double ysize, double zsize, bool ignoreUnknown=false);
		bool randomSample(map_manager::RandomSample::Request &req, map_manager::RandomSample::Response &res);
	};
}

#endif