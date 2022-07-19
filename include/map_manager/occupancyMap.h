/*
	FILE: occupancyMap,h
	-------------------------------------
	occupancy voxel map header file
*/
#ifndef MAPMANAGER_OCCUPANCYMAP
#define MAPMANAGER_OCCUPANCYMAP
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <queue>
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
#include <map_manager/raycast.h>

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
		ros::Publisher mapVisPub_;

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
		double pHitLog_, pMissLog_, pMinLog_, pMaxLog_, pOccLog_; 

		// MAP
		double UNKNOWN_FLAG_ = 0.01;
		double mapRes_;
		double groundHeight_; // ground height in z axis
		Eigen::Vector3d mapSize_, mapSizeMin_, mapSizeMax_; // reserved min/max map size
		Eigen::Vector3i mapVoxelMin_, mapVoxelMax_; // reserved min/max map size in voxel
		Eigen::Vector3d localUpdateRange_; // self defined local update range
		Eigen::Vector3d localMapSize_;
		Eigen::Vector3i localMapVoxel_; // voxel representation of local map size
		double localBoundInflate_;

		// VISUALZATION
		double maxVisHeight_;
		// -----------------------------------------------------------------



		// data
		// -----------------------------------------------------------------
		// SENSOR DATA
		cv::Mat depthImage_;
		Eigen::Vector3d position_; // current position
		Eigen::Matrix3d orientation_; // current orientation
		Eigen::Vector3i localBoundMin_, localBoundMax_; // sensor data range


		// MAP DATA
		int projPointsNum_ = 0;
		std::vector<Eigen::Vector3d> projPoints_; // projected points from depth image
		std::vector<int> countHitMiss_;
		std::vector<int> countHit_;
		std::queue<Eigen::Vector3i> updateVoxelCache_;
		std::vector<double> occupancy_; // occupancy log data
		std::vector<char> occupancyInflated_; // inflated occupancy data
		char raycastNum_ = 0; 
		std::vector<char> flagTraverse_, flagRayend_;

		

		// STATUS
		bool occNeedUpdate_ = false;

		// Raycaster
		RayCaster raycaster_;




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

		// user functions
		bool isOccupied(const Eigen::Vector3d& pos);
		bool isOccupied(const Eigen::Vector3i& idx); // does not count for unknown
		bool isFree(const Eigen::Vector3d& pos);
		bool isFree(const Eigen::Vector3i& idx);
		bool isUnknown(const Eigen::Vector3d& pos);
		bool isUnknown(const Eigen::Vector3i& idx);

		// Visualziation
		void visCB(const ros::TimerEvent& );
		void publishProjPoints();
		void publishMap();


		// helper functions
		bool isInMap(const Eigen::Vector3d& pos);
		bool isInMap(const Eigen::Vector3i& idx);
		void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& idx);
		void indexToPos(const Eigen::Vector3i& idx, Eigen::Vector3d& pos);
		int posToAddress(const Eigen::Vector3d& idx);
		int indexToAddress(const Eigen::Vector3i& idx);
		void boundIndex(Eigen::Vector3i& idx);
		bool isInLocalUpdateRange(const Eigen::Vector3d& pos);
		bool isInLocalUpdateRange(const Eigen::Vector3i& idx);
		Eigen::Vector3d adjustPointInMap(const Eigen::Vector3d& point);
		Eigen::Vector3d adjustPointRayLength(const Eigen::Vector3d& point);
		int updateOccupancyInfo(const Eigen::Vector3d& point, bool isOccupied);
		double logit(double x);
		void getCameraPose(const geometry_msgs::PoseStampedConstPtr& pose, Eigen::Matrix4d& camPoseMatrix);
	
		int setCacheOccupancy(Eigen::Vector3d pos, int occ);
		Eigen::Vector3d closetPointInMap(const Eigen::Vector3d& pt, const Eigen::Vector3d& camera_pt);
	};
}

#endif