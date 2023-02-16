/*
	FILE: dynamicMap.h
	--------------------------------------
	header file dynamic map
*/

#ifndef MAPMANAGER_DYNAMICMAP_H
#define MAPMANAGER_DYNAMICMAP_H

#include <map_manager/occupancyMap.h>
#include <map_manager/detector/dynamicDetector.h>


namespace mapManager{

	class dynamicMap : public occMap{
	private:

	protected:
		std::shared_ptr<mapManager::dynamicDetector> detector_;
		ros::Timer freeMapTimer_;


	public:
		dynamicMap(const ros::NodeHandle& nh);

		// dynamic clean 
		void freeMapCB(const ros::TimerEvent&);

		// user function
		void getDynamicObstacles(std::vector<Eigen::Vector3d>& obstaclePos, 
								 std::vector<Eigen::Vector3d>& obstaclesVel, 
			                     std::vector<Eigen::Vector3d>& obstacleSize);
	};

}

#endif

