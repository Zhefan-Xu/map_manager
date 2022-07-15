/*
	FILE: occupancyMap,h
	-------------------------------------
	occupancy voxel map header file
*/
#ifndef MAPMANAGER_OCCUPANCYMAP
#define MAPMANAGER_OCCUPANCYMAP
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using std::cout; using std::endl;
namespace mapManager{
	class occMap{
	private:

	protected:
		// ROS
		ros::NodeHandle nh_;
		std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depthSub_;
		std::shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> poseSub_;
		typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, geometry_msgs::PoseStamped> depthPoseSync;
		std::shared_ptr<message_filters::Synchronizer<depthPoseSync>> depthPoseSync_;
		ros::Timer occTimer_;
		ros::Timer visTimer_;
		ros::Publisher depthCloudPub_;

		std::string depthTopicName_; // depth image topic
		std::string poseTopicName_;  // pose topic

		// parameters
		// -----------------------------------------------------------------
		// CAMERA
		double fx_, fy_, cx_, cy_; // depth camera intrinsics
		double depthScale_; // value / depthScale
		double depthMinValue_, depthMaxValue_;
		int depthFilterMargin_, skipPixel_; // depth filter margin
		int imgCols_, imgRows_;
		Eigen::Matrix4d body2Cam_; // from body frame to camera frame

		// RAYCASTING
		double raycastMaxLength_;

		// MAP
		double mapRes_;
		Eigen::Vector3d mapSize_, mapSizeMin_, mapSizeMax_; // reserved min/max map size
		Eigen::Vector3i mapVoxelMin_, mapVoxelMax_; // reserved min/max map size in voxel
		// -----------------------------------------------------------------



		// data
		// -----------------------------------------------------------------
		// SENSOR DATA
		cv::Mat depthImage_;
		Eigen::Vector3d position_; // current position
		Eigen::Matrix3d orientation_; // current orientation


		// MAP DATA
		int projPointsNum_ = 0;
		std::vector<Eigen::Vector3d> projPoints_;

		// STATUS
		bool occNeedUpdate_ = false;


		// ------------------------------------------------------------------


	public:
		occMap(); // empty constructor
		void initMap(const ros::NodeHandle& nh);
		void initParam();
		void registerCallback();
		void registerPub();

		// callback
		void depthPoseCB(const sensor_msgs::ImageConstPtr& img, const geometry_msgs::PoseStampedConstPtr& pose);
		void updateOccupancyCB(const ros::TimerEvent& );

		// core function
		void projectDepthImage();
		void raycastUpdate();
		void cleanLocalMap();
		void inflateLocalMap();

		// Visualziation
		void visCB(const ros::TimerEvent& );
		void publishProjPoints();
		bool isInMap(const Eigen::Vector3d& pos);
		bool isInMap(const Eigen::Vector3i& idx);
		void getCameraPose(const geometry_msgs::PoseStampedConstPtr& pose, Eigen::Matrix4d& camPoseMatrix);
	};
}

#endif