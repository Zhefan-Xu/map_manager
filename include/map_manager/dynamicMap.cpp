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

	dynamicMap::dynamicMap(const ros::NodeHandle& nh){
		this->ns_ = "dynamic_map";
		this->hint_ = "[dynamicMap]";
		this->initMap(nh);
	}

	void dynamicMap::initMap(const ros::NodeHandle& nh){
		this->nh_ = nh;
		this->initParam();    
		this->initDynamicParam();

		this->registerPub();
		this->registerDynamicPub();
		this->registerDynamicCallback();
		this->registerCallback();
	}

	void dynamicMap::initDynamicParam(){
		// debuging
		// if (not this->nh_.getParam(this->ns_ + "/detection_debug", this->detectionDebug_)){
		// 	this->detectionDebug_= false;
		// 	cout << this->hint_ << ": No detection debug parameter. Use default: false." << endl;
		// }
		// else{
		// 	cout << this->hint_ << ": Detection debug is set to: " << this->detectionDebug_ << endl;
		// }  

		// point cloud time differnce in terms of #time_stampes
		

		if (not this->nh_.getParam(this->ns_ + "/time_stamp", this->ts_)){
			this->ts_ = 0.05;
			std::cout << this->hint_ << ": No time_stamp parameter. Use default: 5." << std::endl;
		}
		else{
			std::cout << this->hint_ << ": time_stamp is set to: " << this->ts_ << std::endl;
		}


	}


	void dynamicMap::registerDynamicPub(){
		// this->depthCloudPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>(this->ns_ + "/depth_cloud", 10);
		this->depthCloudFilteredPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>(this->ns_ + "/depth_cloud_filt", 2);
		// this->dynamicBoxPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->ns_+"/box_visualization_marker", 10);
		// this->fusedBoxPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->ns_+"/fused_box_visualization_marker", 10);
		// this->rawBoxPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->ns_+"/raw_box_visualization_marker", 10);
		// this->dynamicVelPub_ = this->nh_.advertise<std_msgs::Float64>(this->ns_+"/dynamic_vel", 1);
		// this->dynamicPosPub_ = this->nh_.advertise<geometry_msgs::PointStamped>(this->ns_+"/dynamic_pos", 1);
		// this->obstacleTrajPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->ns_ + "/traj_marker", 10);
	}


	void dynamicMap::registerCallback() {
		this->depthSub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(this->nh_, this->depthTopicName_, 50));
		if (this->localizationMode_ == 0){
			this->poseSub_.reset(new message_filters::Subscriber<geometry_msgs::PoseStamped>(this->nh_, this->poseTopicName_, 25));
			this->depthPoseSync_.reset(new message_filters::Synchronizer<depthPoseSync>(depthPoseSync(100), *this->depthSub_, *this->poseSub_));
			this->depthPoseSync_->registerCallback(boost::bind(&occMap::depthPoseCB, this, _1, _2));
		}
		else if (this->localizationMode_ == 1){
			this->odomSub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(this->nh_, this->odomTopicName_, 25));
			this->depthOdomSync_.reset(new message_filters::Synchronizer<depthOdomSync>(depthOdomSync(100), *this->depthSub_, *this->odomSub_));
			this->depthOdomSync_->registerCallback(boost::bind(&occMap::depthOdomCB, this, _1, _2));
		}
		else{
			ROS_ERROR("[dynamicMap]: Invalid localization mode!");
			exit(0);
		}
   
		// // occupancy update callback
		this->occTimer_ = this->nh_.createTimer(ros::Duration(0.05), &dynamicMap::updateOccupancyCB, this);

		// map inflation callback
		this->inflateTimer_ = this->nh_.createTimer(ros::Duration(0.05), &dynamicMap::inflateMapCB, this);

		// visualization callback
		this->visTimer_ = this->nh_.createTimer(ros::Duration(0.05), &occMap::visCB, dynamic_cast<occMap*>(this));
	}


	void dynamicMap::inflateMapCB(const ros::TimerEvent&){   
		// inflate local map:
		if (this->mapNeedInflate_){
			this->inflateLocalMap();
			this->mapNeedInflate_ = false;
			this->esdfNeedUpdate_ = true;
		}
	}


	void dynamicMap::updateOccupancyCB(const ros::TimerEvent&){
		if (not this->occNeedUpdate_){
			return;
		}
		// cout << "update occupancy map" << endl;
		ros::Time startTime, endTime;
		
		startTime = ros::Time::now();
		// project 3D points from depth map
		this->projectDepthImage();
		ROS_INFO("in updataOccCB");
		std::cout<<"projpoints: "<<this->projPoints_.size()<<std::endl;
		// raycasting and update occupancy
		this->raycastUpdate();

		// clear local map
		if (this->cleanLocalMap_){
			this->cleanLocalMap();
		}
		
		// infalte map
		endTime = ros::Time::now();
		if (this->verbose_){
			cout << this->hint_ << ": Occupancy update time: " << (endTime - startTime).toSec() << " s." << endl;
		}
		this->occNeedUpdate_ = false;
		this->mapNeedInflate_ = true;
	}



	void dynamicMap::registerDynamicCallback(){
		this->detector_.reset(new mapManager::dynamicDetector(this->nh_, this->projPoints_));
		this->detector_->initDetectorParam();
		
		this->dynamicObsDetectTimer_ = this->nh_.createTimer(ros::Duration(this->ts_), &dynamicMap::dynamicObsDetectCB, this);
		this->visTimer_ = this->nh_.createTimer(ros::Duration(this->ts_), &dynamicMap::visDynamicCB, this);
		// this->detector_.reset(new mapManager::boxDetector(this->nh_, this->ns_,this->depthImage_, this->fx_,this->fy_,this->cx_,this->cy_, this->depthScale_));
		// this->boxDetectTimer_ = this->nh_.createTimer(ros::Duration(this->ts_), &dynamicMap::boxDetectCB, this);
		// this->dynamicBoxPubTimer_ = this->nh_.createTimer(ros::Duration(this->ts_), &dynamicMap::dynamicBoxPubCB, this);
		// this->obstacleTrajPubTimer_ = this->nh_.createTimer(ros::Duration(this->ts_),&dynamicMap::obstacleTrajPubCB, this);
		
	}

	void dynamicMap::dynamicObsDetectCB(const ros::TimerEvent&){
		this->detector_->filteringAndClustering();
	}

	void dynamicMap::visDynamicCB(const ros::TimerEvent&){
		this->publishFilteredPoinCloud();
	}

	void dynamicMap::publishFilteredPoinCloud(){
		pcl::PointXYZ pt;
		pcl::PointCloud<pcl::PointXYZ> cloud;
		std::vector<Eigen::Vector3d> filteredPc;
		ROS_INFO("before got filtered pc");
		this->detector_->getFilteredPc(filteredPc);
		ROS_INFO("got filtered pc: %li", filteredPc.size());
		for (size_t i=0; i<filteredPc.size(); ++i){
			pt.x = filteredPc[i](0);
			pt.y = filteredPc[i](1);
			pt.z = filteredPc[i](2);
			cloud.push_back(pt);
		}

		cloud.width = cloud.points.size();
		cloud.height = 1;
		cloud.is_dense = true;
		cloud.header.frame_id = "map";

		sensor_msgs::PointCloud2 cloudMsg;
		pcl::toROSMsg(cloud, cloudMsg);
		ROS_INFO("publish");
		this->depthCloudFilteredPub_.publish(cloudMsg);
	}


	// void dynamicMap::registerCallback() {
	// 	this->depthSub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(this->nh_, this->depthTopicName_, 50));
	// 	if (this->localizationMode_ == 0){
	// 		this->poseSub_.reset(new message_filters::Subscriber<geometry_msgs::PoseStamped>(this->nh_, this->poseTopicName_, 25));
	// 		this->depthPoseSync_.reset(new message_filters::Synchronizer<depthPoseSync>(depthPoseSync(100), *this->depthSub_, *this->poseSub_));
	// 		this->depthPoseSync_->registerCallback(boost::bind(&occMap::depthPoseCB, this, _1, _2));
	// 	}
	// 	else if (this->localizationMode_ == 1){
	// 		this->odomSub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(this->nh_, this->odomTopicName_, 25));
	// 		this->depthOdomSync_.reset(new message_filters::Synchronizer<depthOdomSync>(depthOdomSync(100), *this->depthSub_, *this->odomSub_));
	// 		this->depthOdomSync_->registerCallback(boost::bind(&occMap::depthOdomCB, this, _1, _2));
	// 	}
	// 	else{
	// 		ROS_ERROR("[dynamicMap]: Invalid localization mode!");
	// 		exit(0);
	// 	}
   
	// 	// occupancy update callback
	// 	this->occTimer_ = this->nh_.createTimer(ros::Duration(0.05), &dynamicMap::updateOccupancyCB, this);

	// 	// map inflation callback
	// 	this->inflateTimer_ = this->nh_.createTimer(ros::Duration(0.05), &dynamicMap::inflateMapCB, this);

	// 	// visualization callback
	// 	this->visTimer_ = this->nh_.createTimer(ros::Duration(0.05), &occMap::visCB, dynamic_cast<occMap*>(this));
	// }

}

