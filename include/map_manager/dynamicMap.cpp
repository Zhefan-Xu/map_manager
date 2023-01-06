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

		

	}


	void dynamicMap::registerDynamicPub(){
		// this->dynamicBoxPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->ns_+"/box_visualization_marker", 10);
		// this->fusedBoxPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->ns_+"/fused_box_visualization_marker", 10);
		// this->rawBoxPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->ns_+"/raw_box_visualization_marker", 10);
		// this->dynamicVelPub_ = this->nh_.advertise<std_msgs::Float64>(this->ns_+"/dynamic_vel", 1);
		// this->dynamicPosPub_ = this->nh_.advertise<geometry_msgs::PointStamped>(this->ns_+"/dynamic_pos", 1);
		// this->obstacleTrajPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->ns_ + "/traj_marker", 10);
	}


	void dynamicMap::registerDynamicCallback(){
		// this->detector_.reset(new mapManager::boxDetector(this->nh_, this->ns_,this->depthImage_, this->fx_,this->fy_,this->cx_,this->cy_, this->depthScale_));
		// this->boxDetectTimer_ = this->nh_.createTimer(ros::Duration(this->ts_), &dynamicMap::boxDetectCB, this);
		// this->dynamicBoxPubTimer_ = this->nh_.createTimer(ros::Duration(this->ts_), &dynamicMap::dynamicBoxPubCB, this);
		// this->obstacleTrajPubTimer_ = this->nh_.createTimer(ros::Duration(this->ts_),&dynamicMap::obstacleTrajPubCB, this);
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

