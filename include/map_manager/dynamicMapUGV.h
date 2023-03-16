/*
	FILE: dynamicMapUGV.h
	--------------------------------------
	header file dynamic map
*/

#ifndef MAPMANAGER_DYNAMICMAPUGV_H
#define MAPMANAGER_DYNAMICMAPUGV_H

#include <map_manager/occupancyMap.h>
#include <map_manager/detector/dynamicDetector.h>
#include <vision_msgs/BoundingBox3D.h>
#include <vision_msgs/Detection3DArray.h>

#include <math.h>

namespace mapManager{

	class dynamicMapUGV : public occMap{
	private:

	protected:

		// param
		std::string lidarDetectTopicName_;

		// members
		std::shared_ptr<mapManager::dynamicDetector> detector_;
		ros::Timer freeMapTimer_;
		ros::Subscriber lidarSub_ ;
		ros::Publisher mergedLidarBoxesPub_;

        std::vector<box3D> lidarBoxes_;
        std::vector<std::deque<box3D>> lidarBoxesHist_;
        


	public:
		dynamicMapUGV();
		dynamicMapUGV(const ros::NodeHandle& nh);

		
		void initMap(const ros::NodeHandle& nh);

		void initDynamicUGVParam();

		void registerPub();

		void visCB(const ros::TimerEvent& );
		
		// dynamic clean 
		void freeMapCB(const ros::TimerEvent&);

        // get lidar box 
        void lidarBoxCB(const vision_msgs::Detection3DArrayConstPtr& box1_msg); // This currently works 

		void findBestMatch(std::vector<int> &bestMatch);

		// user function
		void getDynamicObstacles(std::vector<Eigen::Vector3d>& obstaclePos, 
								 std::vector<Eigen::Vector3d>& obstaclesVel, 
			                     std::vector<Eigen::Vector3d>& obstacleSize);
        
        // tool functions
        bool isInFov(const box3D& box, const Eigen::Vector3d& position , const Eigen::Matrix3d& orientation);
		void publish3dBox(const std::vector<box3D>& boxes, const ros::Publisher& publisher, double r, double g, double b);
	};

}

#endif

