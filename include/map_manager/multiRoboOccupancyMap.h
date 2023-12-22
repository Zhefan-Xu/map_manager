/*
	FILE: occupancyMap,h
	-------------------------------------
	occupancy voxel map header file
*/
#ifndef MAPMANAGER_MULTIROBO_OCCUPANCYMAP
#define MAPMANAGER_MULTIROBO_OCCUPANCYMAP
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <queue>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <map_manager/raycast.h>
#include <map_manager/sharedVoxels.h>
#include <map_manager/robotStates.h>
#include <map_manager/occupancyMap.h>
#include <thread>


namespace mapManager{
	class multiRoboOccMap : occMap{
	private:
		// ROS
		ros::NodeHandle nh_;
		ros::Publisher mapSharedPub_;
		ros::Subscriber mapSharedSub_;
		ros::Publisher robotStatesPub_;
		ros::Subscriber robotStatesSub_;
		ros::Timer mapSharedPubTimer_;


		// PARAMS
		int robot_id_;
		int robot_num_;
		Eigen::Matrix4d global2Map_; // from robot team global frame to map frame 

		// STATUS DATA	
		std::vector<int> readyRobotsID_; // vector recording IDs of robot being ready to share map
		bool allRobotsReady_; // all robots in the network are ready to share map
		map_manager::sharedVoxels sharedVoxels_; 
		map_manager::robotStates robotStates_;

	public:
		multiRoboOccMap();
		multiRoboOccMap(const ros::NodeHandle& nh);
		virtual ~multiRoboOccMap() = default;
		void initMultiRoboOccMap(const ros::NodeHandle& nh);
		void initMultiRoboParam();
		void registerMultiRoboCallback();
		void registerMultiRoboPub();
		void depthPoseCB(const sensor_msgs::ImageConstPtr& img, const geometry_msgs::PoseStampedConstPtr& pose);
		void depthOdomCB(const sensor_msgs::ImageConstPtr& img, const nav_msgs::OdometryConstPtr& odom);
		void pointcloudPoseCB(const sensor_msgs::PointCloud2ConstPtr& pointcloud, const geometry_msgs::PoseStampedConstPtr& pose);
		void pointcloudOdomCB(const sensor_msgs::PointCloud2ConstPtr& pointcloud, const nav_msgs::OdometryConstPtr& odom);
		void mapSharedPubCB(const ros::TimerEvent& );
		void mapSharedSubCB(const map_manager::sharedVoxelsConstPtr& incomeVoxels);
		void robotStatesSubCB(const map_manager::robotStatesConstPtr& states);
		void updateOccupancyCB(const ros::TimerEvent& );

		// helper functions
		void raycastUpdate();
		// void updateSharedVoxels(const map_manager::sharedVoxels& sharedVoxels);
		// void updateRobotStates(const map_manager::robotStates& states);
		// void updateSharedVoxels(const std::vector<int>& robotIDs, const std::vector<std::vector<int>>& sharedVoxels);

	};
}
#endif
