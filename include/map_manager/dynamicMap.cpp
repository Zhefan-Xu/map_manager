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
		if (not this->nh_.getParam(this->ns_ + "/detection_debug", this->detectionDebug_)){
			this->detectionDebug_= false;
			cout << this->hint_ << ": No detection debug parameter. Use default: false." << endl;
		}
		else{
			cout << this->hint_ << ": Detection debug is set to: " << this->detectionDebug_ << endl;
		}  

		if (not this->nh_.getParam(this->ns_ + "/velocity_debug", this->veDebug_)){
			this->veDebug_= false;
			cout << this->hint_ << ": No velocity debug parameter. Use default: false." << endl;
		}   
		else{
			cout << this->hint_ << ": Velocity debug is set to: " << this->veDebug_ << endl;
		}


		// testing
		if (not this->nh_.getParam(this->ns_ + "/publish_test_vel", this->testVel_)){
			this->testVel_= false;
			cout << this->hint_ << ": No publish test velocity parameter. Use default: false." << endl;
		}
		else{
			cout << this->hint_ << ": Publish test velocity is set to: " << this->testVel_ << endl;
		}

		if (not this->nh_.getParam(this->ns_ + "/publish_test_pos", this->testPos_)){
			this->testPos_= false;
			cout << this->hint_ << ": No publish test position parameter. Use default: false." << endl;
		}
		else{
			cout << this->hint_ << ": Publish test position is set to: " << this->testPos_ << endl;
		}

		// map refinement inflation coefficient
		if (not this->nh_.getParam(this->ns_ + "/map_refinement_inflation_coefficient", this->mapRefineInflateCoef_)){
			this->mapRefineInflateCoef_ = 1.15;
			cout << this->hint_ << ": No map refinement inflation coefficient. Use default: 1.15." << endl;
		}
		else{
			cout << this->hint_ << ": Map refinement inflation coefficient: " << this->mapRefineInflateCoef_ << endl;
		}
		
		// raw bounding box width limit lower
		if (not this->nh_.getParam(this->ns_ + "/raw_box_width_limit_lower", this->rawBoxWidthLimitLower_)){
			this->rawBoxWidthLimitLower_ = 0.2;
			cout << this->hint_ << ": No raw box width limit lower. Use default: 0.2." << endl;
		}
		else{
			cout << this->hint_ << ": Raw box width limit lower: " << this->rawBoxWidthLimitLower_ << endl;
		}

		if (not this->nh_.getParam(this->ns_ + "/raw_box_height_limit_lower", this->rawBoxHeightLimitLower_)){
			this->rawBoxHeightLimitLower_ = 0.5;
			cout << this->hint_ << ": No raw box height limit lower. Use default: 0.5." << endl;
		}
		else{
			cout << this->hint_ << ": Raw box height limit lower: " << this->rawBoxHeightLimitLower_ << endl;
		}

		if (not this->nh_.getParam(this->ns_ + "/raw_box_width_limit_upper", this->rawBoxWidthLimitUpper_)){
			this->rawBoxWidthLimitUpper_ = 1.3;
			cout << this->hint_ << ": No raw box width limit upper. Use default: 1.3." << endl;
		}
		else{
			cout << this->hint_ << ": Raw box width limit upper: " << this->rawBoxWidthLimitUpper_ << endl;
		}

		if (not this->nh_.getParam(this->ns_ + "/raw_box_height_limit_upper", this->rawBoxHeightLimitUpper_)){
			this->rawBoxHeightLimitUpper_ = 2.0;
			cout << this->hint_ << ": No raw box height limit upper. Use default: 2.0." << endl;
		}
		else{
			cout << this->hint_ << ": Raw box heigt limit upper: " << this->rawBoxHeightLimitUpper_ << endl;
		}

		if (not this->nh_.getParam(this->ns_ + "/map_refine_floor", this->mapRefineFloor_)){
			this->mapRefineFloor_= 0.2;
			cout << this->hint_ << ": No map refine floor height. Use default: 0.2." << endl;
		}
		else{
			cout << this->hint_ << ": Map refine floor height: " << this->mapRefineFloor_ << endl;
		}

		int preHistCleanSizeTemp;
		if (not this->nh_.getParam(this->ns_ + "/pre_history_clean_size", preHistCleanSizeTemp)){
			this->preHistCleanSize_ = 100;
			cout << this->hint_ << ": No dynamic region clean frame size. Use default: 100." << endl;
		}
		else{
			this->preHistCleanSize_ = preHistCleanSizeTemp;
			cout << this->hint_ << ": Dynamic region clean frame size: " << this->preHistCleanSize_ << endl;
		}

		if (not this->nh_.getParam(this->ns_ + "/hist_clean_skip", this->histCleanSkip_)){
			this->histCleanSkip_= 1;
			cout << this->hint_ << ": No history map dynamic region clean skip. Use default: 1." << endl;
		}
		else{
			cout << this->hint_ << ": History map dynamic region clean: " << this->histCleanSkip_ << endl;
		}

		if (not this->nh_.getParam(this->ns_ + "/clean_extra_dist", this->cleanExtraDist_)){
			this->cleanExtraDist_= 5;
			cout << this->hint_ << ": No vertical extra clean distance. Use default: 5." << endl;
		}
		else{
			cout << this->hint_ << ": Vertical extra clean distance: " << this->cleanExtraDist_ << endl;
		}

		if (not this->nh_.getParam(this->ns_ + "/ground_thickness", this->groundThick_)){
			this->groundThick_= 0.1;
			cout << this->hint_ << ": No ground thickness parameter. Use default: 0.1." << endl;
		}
		else{
			cout << this->hint_ << ": Ground thickness: " << this->groundThick_ << endl;
		}

		if (not this->nh_.getParam(this->ns_ + "/timestep", this->ts_)){
			this->ts_ = 0.033;
			cout << this->hint_ << ": No timestep. Use default: 0.033." << endl;
		}
		else{
			cout << this->hint_ << ": Timestep: " << this->ts_ << endl;
		}

		if (not this->nh_.getParam(this->ns_ + "/overlap_threshold", this->overlapThreshold_)){
			this->overlapThreshold_ = 0.3;
			cout << this->hint_ << ": No overlap threshold. Use default: 0.3." << endl;
		}
		else{
			cout << this->hint_ << ": Overlap threshold: " << this->overlapThreshold_ << endl;
		}

		if (not this->nh_.getParam(this->ns_ + "/continuity_thresh", this->contTresh_)){
			this->contTresh_= 0.1;
			cout << this->hint_ << ": No continuity threshold. Use default: 0.1." << endl;
		}
		else{
			cout << this->hint_ << ": Continuity threshold: " << this->contTresh_ << endl;
		}

		if (not this->nh_.getParam(this->ns_ + "/velocity_threshold", this->vAvgEstimatedMetric_)){
			this->vAvgEstimatedMetric_ = 0.4;
			cout << this->hint_ << ": No velocity threshold. Use default: 0.4." << endl;
		}
		else{
			cout << this->hint_ << ": Velocity threshold: " << this->vAvgEstimatedMetric_ << endl;
		}

		int fovXMargin;
		if (not this->nh_.getParam(this->ns_ + "/fov_x_margin", fovXMargin)){
			fovXMargin = 3;
			this->fovXMarginLower_ = this->imgCols_ - fovXMargin;
			this->fovXMarginUpper_ = this->imgCols_ + fovXMargin;
			cout << this->hint_ << ": No FOV horizontal margin. Use default: 3." << endl;
		}
		else{
			this->fovXMarginLower_ = this->imgCols_ - fovXMargin;
			this->fovXMarginUpper_ = this->imgCols_ + fovXMargin;
			cout << this->hint_ << ": FOV horizontal margin: " << fovXMargin << endl;
		}

		int fovYMargin;
		if (not this->nh_.getParam(this->ns_ + "/fov_y_margin", fovYMargin)){
			fovYMargin = 0;
			this->fovYMarginLower_ = this->imgRows_ - fovYMargin;
			this->fovYMarginUpper_ = this->imgRows_ + fovYMargin;
			cout << this->hint_ << ": No FOV vertical margin. Use default: 5." << endl;
		}
		else{
			this->fovYMarginLower_ = this->imgRows_ - fovYMargin;
			this->fovYMarginUpper_ = this->imgRows_ + fovYMargin;
			cout << this->hint_ << ": FOV vertical margin: " << fovYMargin << endl;
		}

		if (not this->nh_.getParam(this->ns_ + "/e_p", this->eP_)){
			this->eP_= 0.3;
			cout << this->hint_ << ": No e_p. Use default: 0.3." << endl;
		}
		else{
			cout << this->hint_ << ": e_p: " << this->eP_ << endl;
		}
		if (not this->nh_.getParam(this->ns_ + "/e_q", this->eQ_)){
			this->eQ_= 0.002;
			cout << this->hint_ << ": No e_q. Use default: 0.002." << endl;
		}
		else{
			cout << this->hint_ << ": e_q: " << this->eQ_ << endl;
		}
		if (not this->nh_.getParam(this->ns_ + "/e_r", this->eR_)){
			this->eR_= 0.3;
			cout << this->hint_ << ": No e_r. Use default: 0.3." << endl;
		}
		else{
			cout << this->hint_ << ": e_r: " << this->eR_ << endl;
		}


		int avgVHistSizeTemp;
		if (not this->nh_.getParam(this->ns_ + "/avg_velocty_history_size", avgVHistSizeTemp)){
			this->avgVHistSize_= 5;
			cout << this->hint_ << ": No number of frames to estimate velocity parameter. Use default: 5." << endl;
		}
		else{
			this->avgVHistSize_ = avgVHistSizeTemp;
			cout << this->hint_ << ": Number of frames to estimate velocity: " << this->avgVHistSize_ << endl;
		}

		int nowHistSizeTemp;
		if (not this->nh_.getParam(this->ns_ + "/now_history_size", nowHistSizeTemp)){
			this->nowHistSize_= 20;
			cout << this->hint_ << ": No size of tracking history parameter. Use default: 20." << endl;
		}
		else{
			this->nowHistSize_ = nowHistSizeTemp;
			cout << this->hint_ << ": Size of tracking history: " << this->nowHistSize_ << endl;
		}

		int CFHistSizeTemp;
		if (not this->nh_.getParam(this->ns_ + "/continuity_filter_history_size", CFHistSizeTemp)){
			this->CFHistSize_= 10;
			cout << this->hint_ << ": No continuity filter frame size. Use default: 10." << endl;
		}
		else{
			this->CFHistSize_ = CFHistSizeTemp;
			cout << this->hint_ << ": Continuity filter frame size: " << this->CFHistSize_ << endl;
		}

		int CFIntervalTemp;
		if (not this->nh_.getParam(this->ns_ + "/continuity_filter_interval", CFIntervalTemp)){
			this->CFInterval_= 4;
			cout << this->hint_ << ": No continurity pairing interval. Use default: 4." << endl;
		}
		else{
			this->CFInterval_ = CFIntervalTemp;
			cout << this->hint_ << ": Continuity pairing interval: " << this->CFInterval_ << endl;
		}

		if (not this->nh_.getParam(this->ns_ + "/size_ratio_lower", this->sizeRatioLower_)){
			this->sizeRatioLower_ = 0.2;
			cout << this->hint_ << ": No size ratio lower. Use default: 0.2." << endl;
		}
		else{
			cout << this->hint_ << ": Size ratio lower: " << this->sizeRatioLower_ << endl;
		}

		if (not this->nh_.getParam(this->ns_ + "/size_ratio_upper", this->sizeRatioUpper_)){
			this->sizeRatioUpper_ = 0.7;
			cout << this->hint_ << ": No size ratio upper. Use default: 0.7." << endl;
		}
		else{
			cout << this->hint_ << ": Size ratio upper: " << this->sizeRatioUpper_ << endl;
		}

		if (not this->nh_.getParam(this->ns_ + "/fused_box_width_limit_lower", this->fusedBoxWidthLimitLower_)){
			this->fusedBoxWidthLimitLower_ = 0.25;
			cout << this->hint_ << ": No fused box width limit lower. Use default: 0.25." << endl;
		}
		else{
			cout << this->hint_ << ": Fused box width limit lower: " << this->fusedBoxWidthLimitLower_ << endl;
		}

		if (not this->nh_.getParam(this->ns_ + "/fused_box_height_limit_lower", this->fusedBoxHeightLimitLower_)){
			this->fusedBoxHeightLimitLower_ = 1.1;
			cout << this->hint_ << ": No fused box height limit lower. Use default: 1.1." << endl;
		}
		else{
			cout << this->hint_ << ": Fused box height limit lower: " << this->fusedBoxHeightLimitLower_ << endl;
		}

		if (not this->nh_.getParam(this->ns_ + "/fused_box_width_limit_upper", this->fusedBoxWidthLimitUpper_)){
			this->fusedBoxWidthLimitUpper_ = 1.2;
			cout << this->hint_ << ": No fused box width limit upper. Use default: 1.2." << endl;
		}
		else{
			cout << this->hint_ << ": Fused box width limit upper: " << this->fusedBoxWidthLimitUpper_ << endl;
		}

		if (not this->nh_.getParam(this->ns_ + "/fused_box_height_limit_upper", this->fusedBoxHeightLimitUpper_)){
			this->fusedBoxHeightLimitUpper_ = 2.0;
			cout << this->hint_ << ": No fused box height limit upper. Use default: 2.0." << endl;
		}
		else{
			cout << this->hint_ << ": Fused box height limit upper: " << this->fusedBoxHeightLimitUpper_ << endl;
		}

		if (not this->nh_.getParam(this->ns_ + "/lowest_point_limit", this->lowestPointLimit_)){
			this->lowestPointLimit_ = 0.2;
			cout << this->hint_ << ": No lowest point limit. Use default: 0.2." << endl;
		}
		else{
			cout << this->hint_ << ": Lowest point limit: " << this->lowestPointLimit_ << endl;
		}

		if (not this->nh_.getParam(this->ns_ + "/static_thresh", this->staticThresh_)){
			this->staticThresh_= 4;
			cout << this->hint_ << ": No static. Use default: 4." << endl;
		}
		else{
			cout << this->hint_ << ": static: " << this->staticThresh_ << endl;
		}

		if (not this->nh_.getParam(this->ns_ + "/dynamic_thresh", this->dynamicThresh_)){
			this->dynamicThresh_= 8;
			cout << this->hint_ << ": No dynamic_thresh. Use default: 8." << endl;
		}
		else{
			cout << this->hint_ << ": dynamic_thresh: " << this->dynamicThresh_ << endl;
		}

		int voteHistSizeTemp;
		if (not this->nh_.getParam(this->ns_ + "/vote_history_size", voteHistSizeTemp)){
			this->voteHistSize_= 10;
			cout << this->hint_ << ": No vote_history_size. Use default: 5." << endl;
		}
		else{
			this->voteHistSize_ = voteHistSizeTemp;
			cout << this->hint_ << ": vote_history_size: " << this->voteHistSize_ << endl;
		}

		if (not this->nh_.getParam(this->ns_ + "/n", this->n_)){
			this->n_ = 5;
			cout << this->hint_ << ": No number of paths in library. Use default: 2.0." << endl;
		}
		else{
			cout << this->hint_ << ": Number of paths in library: " << this->n_ << endl;
		}

		if (not this->nh_.getParam(this->ns_ + "/t", this->t_)){
			this->t_ = 2;
			cout << this->hint_ << ": No maximum time to perform collision checking parameter. Use default: 2." << endl;
		}
		else{
			cout << this->hint_ << ": Maximum time to perform collision checking: " << this->t_ << endl;
		}

		if (not this->nh_.getParam(this->ns_ + "/colli_check_end_dist", this->colliCheckEndDist_)){
			this->colliCheckEndDist_ = 3.0;
			cout << this->hint_ << ": No max distance to check collision parameter. Use default: 3.0." << endl;
		}
		else{
			cout << this->hint_ << ": Max distance to check collision: " << this->colliCheckEndDist_ << endl;
		}

		if (not this->nh_.getParam(this->ns_ + "/colli_check_start_dist", this->colliCheckStartDist_)){
			this->colliCheckStartDist_ = 1.0;
			cout << this->hint_ << ": No collision check starting distance parameter. Use default: 1.0." << endl;
		}
		else{
			cout << this->hint_ << ": Collision checking starting distance: " << this->colliCheckStartDist_ << endl;
		}

		std::vector<float> tempVector;
		Eigen::MatrixXd PT(this->n_, this->n_);
		if (not this->nh_.getParam(this->ns_ + "/P_T", tempVector)){
			// PT << 	1, 		0.7499, 0.3162, 0.075, 	0.01,
			// 		0.7499, 1, 		0.7499, 0.3162, 0.075,
			// 		0.3162, 0.7499, 1, 		0.7499, 0.3162,
			// 		0.075, 	0.3162, 0.7499, 1, 		0.7499,
			// 		0.01, 	0.075, 	0.3162, 0.7499, 1;
			// then multiply k and softmax
			PT << 	0.4637, 0.2812, 0.1182, 0.0729, 0.064,
					0.3004, 0.4955, 0.3004, 0.1262, 0.0789,
					0.1369, 0.3258, 0.5373, 0.3258, 0.1369,
					0.0789, 0.1262, 0.3004, 0.4955, 0.3004,
					0.064, 	0.0729, 0.1182, 0.2812, 0.4637;
			this->PT_ = PT;
		}
		else{
			std::cout<<tempVector.size()<<std::endl;
			for (int i=0 ; i<PT.rows() ; i++){
				for (int j=0 ; j<PT.cols() ; j++){
					PT(i,j) = tempVector[i*PT.cols()+j];
					std::cout<<tempVector[i*PT.cols()+j]<<std::endl;
				}
			}
			this->PT_ = PT;
		}
		Eigen::MatrixXd PI(1, this->n_);
		tempVector.clear();
		if (not this->nh_.getParam(this->ns_ + "/P_I", tempVector)){
			// PT << 	1, 		0.7499, 0.3162, 0.075, 	0.01,
			// 		0.7499, 1, 		0.7499, 0.3162, 0.075,
			// 		0.3162, 0.7499, 1, 		0.7499, 0.3162,
			// 		0.075, 	0.3162, 0.7499, 1, 		0.7499,
			// 		0.01, 	0.075, 	0.3162, 0.7499, 1;
			// then multiply k and softmax
			PI << 	0.3162, 0.7499, 1, 0.7499, 0.3162;
			this->PI_ = PI;		}
		else{
			// std::cout<<tempVector.size()<<std::endl;
			for (int i=0 ; i<PI.rows() ; i++){
				for (int j=0 ; j<PI.cols() ; j++){
					PI(i,j) = tempVector[i*PI.cols()+j];
					// std::cout<<tempVector[i*PI.cols()+j]<<std::endl;
				}
			}
			this->PI_ = PI;
		}

		// trajectory prediction gaussian kernel
		MatrixXd C1(7,1);
		C1 << 6.8813, -10.987, -2.5552, 14.248, -8.5216, 2.3566, -0.00125;
		MatrixXd C2(7,1);
		C2 << -0.00083222, 0.01506117, -0.10415035, 0.34858333, -0.59971534, 0.60071107, 0.00129707;
		this->coefs_.push_back(C2);
		this->coefs_.push_back(C1);

	}


	void dynamicMap::registerDynamicPub(){
		this->dynamicBoxPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->ns_+"/box_visualization_marker", 10);
		this->fusedBoxPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->ns_+"/fused_box_visualization_marker", 10);
		this->rawBoxPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->ns_+"/raw_box_visualization_marker", 10);
		this->dynamicVelPub_ = this->nh_.advertise<std_msgs::Float64>(this->ns_+"/dynamic_vel", 1);
		this->dynamicPosPub_ = this->nh_.advertise<geometry_msgs::PointStamped>(this->ns_+"/dynamic_pos", 1);
		this->obstacleTrajPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->ns_ + "/traj_marker", 10);
	}


	void dynamicMap::registerDynamicCallback(){
		this->detector_.reset(new mapManager::boxDetector(this->nh_, this->ns_,this->depthImage_, this->fx_,this->fy_,this->cx_,this->cy_, this->depthScale_));
		this->boxDetectTimer_ = this->nh_.createTimer(ros::Duration(this->ts_), &dynamicMap::boxDetectCB, this);
		this->dynamicBoxPubTimer_ = this->nh_.createTimer(ros::Duration(this->ts_), &dynamicMap::dynamicBoxPubCB, this);
		this->obstacleTrajPubTimer_ = this->nh_.createTimer(ros::Duration(this->ts_),&dynamicMap::obstacleTrajPubCB, this);
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
   
		// occupancy update callback
		this->occTimer_ = this->nh_.createTimer(ros::Duration(0.05), &dynamicMap::updateOccupancyCB, this);

		// map inflation callback
		this->inflateTimer_ = this->nh_.createTimer(ros::Duration(0.05), &dynamicMap::inflateMapCB, this);

		// visualization callback
		this->visTimer_ = this->nh_.createTimer(ros::Duration(0.05), &occMap::visCB, dynamic_cast<occMap*>(this));
	}


	void dynamicMap::inflateMapCB(const ros::TimerEvent&){   
		// inflate local map:
		if (this->mapNeedInflate_){
			this->inflateLocalMap(this->nowHistClean_, this->dynamicHistInd_, this->dynamicObjs_);
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


	void dynamicMap::boxDetectCB(const ros::TimerEvent&){
		// get latest uv detector result
		this->detector_->getBBox(this->depthImage_, this->position_, this->orientation_);
		// function
		this->mapRefine();
		// update box_
		this->trackAndIdentifyDynamic();
		// predict traj
		this->predictTrajectory();
	}


	void dynamicMap::obstacleTrajPubCB(const ros::TimerEvent&){
		// this->publishAllTraj();
		this->publishBestTraj();
	}


	void dynamicMap::dynamicBoxPubCB(const ros::TimerEvent&){

		this->publish3dBox(this->rawBox3Ds_, this->rawBoxPub_, 'g');
		this->publish3dBox(this->fusedBox3Ds_, this->fusedBoxPub_, 'r');
		this->publish3dBox(this->dynamicObjs_, this->dynamicBoxPub_, 'b');
		// test vel or pos
		if (this->testVel_ or this->testPos_){
			this->publishVelAndPos(this->dynamicObjs_);	
		}
	}


	void dynamicMap::inflateLocalMap(const std::deque<std::deque<box3D>> &nowHistClean, const std::vector<int> &dynamicHistInd, const std::vector<box3D> &dynamicObjs){
		Eigen::Vector3i clearIndex;
		// this->occAddressBeforeClean_.clear();
	
		// clear previous data in current data range
		for (int x=this->localBoundMin_(0); x<=this->localBoundMax_(0); ++x){
			for (int y=this->localBoundMin_(1); y<=this->localBoundMax_(1); ++y){
				for (int z=this->localBoundMin_(2); z<=this->localBoundMax_(2); ++z){
					clearIndex(0) = x; clearIndex(1) = y; clearIndex(2) = z;
					this->occupancyInflated_[this->indexToAddress(clearIndex)] = false;
				}
			}
		}

		int xInflateSize = ceil(this->robotSize_(0)/(2*this->mapRes_));
		int yInflateSize = ceil(this->robotSize_(1)/(2*this->mapRes_));
		int zInflateSize = ceil(this->robotSize_(2)/(2*this->mapRes_));

		// inflate based on current occupancy
		Eigen::Vector3i pointIndex, inflateIndex;
		int inflateAddress;
		const int  maxIndex = this->mapVoxelMax_(0) * this->mapVoxelMax_(1) * this->mapVoxelMax_(2);

		// dynamic region clean
		this->dynamicRegionClean(nowHistClean, dynamicHistInd, dynamicObjs);

		// inflate
		for (int x=this->localBoundMin_(0); x<=this->localBoundMax_(0); ++x){
			for (int y=this->localBoundMin_(1); y<=this->localBoundMax_(1); ++y){
				for (int z=this->localBoundMin_(2); z<=this->localBoundMax_(2); ++z){
					pointIndex(0) = x; pointIndex(1) = y; pointIndex(2) = z;
					if (this->isOccupied(pointIndex)){
						for (int ix=-xInflateSize; ix<=xInflateSize; ++ix){
							for (int iy=-yInflateSize; iy<=yInflateSize; ++iy){
								for (int iz=-zInflateSize; iz<=zInflateSize; ++iz){
									inflateIndex(0) = pointIndex(0) + ix;
									inflateIndex(1) = pointIndex(1) + iy;
									inflateIndex(2) = pointIndex(2) + iz;
									inflateAddress = this->indexToAddress(inflateIndex);
									if ((inflateAddress < 0) or (inflateAddress > maxIndex)){
										continue; // those points are not in the reserved map
									} 
									this->occupancyInflated_[inflateAddress] = true;
								}
							}
						}
					}
				}
			}
		}

		
	}


	void dynamicMap::dynamicRegionClean(const std::deque<std::deque<box3D>> &nowHistClean, const std::vector<int> &dynamicHistInd, const std::vector<box3D> &dynamicObjs){
		box3D now;
		box3D pre;

		for (size_t i=0; i<dynamicObjs.size() ; i++) {
			for (size_t j=0; j<nowHistClean[dynamicHistInd[i]].size() ; j+=this->histCleanSkip_) {
				now = nowHistClean[dynamicHistInd[i]][j];

				if (distance<float>(pre.x,pre.y,now.x,now.y) >= 0.1 || j==0) {
					this->cleanSingleFrame(now);
				}
				else {
					continue;
				}		
				pre = now;
			}
		}
	}


	void dynamicMap::cleanSingleFrame(const box3D &dynamicObj) {

		float widthX = dynamicObj.x_width * this->mapRefineInflateCoef_;
		float widthY = dynamicObj.y_width * this->mapRefineInflateCoef_;
		float widthZ = dynamicObj.z_width * this->mapRefineInflateCoef_;
		// const int  maxIndex = this->mapVoxelMax_(0) * this->mapVoxelMax_(1) * this->mapVoxelMax_(2);
		for (float x=dynamicObj.x-widthX/2; x<=dynamicObj.x+widthX/2; x+=this->mapRes_){
			for (float y=dynamicObj.y-widthY/2; y<=dynamicObj.y+widthY/2; y+=this->mapRes_){
				for (float z=dynamicObj.z-widthZ/2-this->cleanExtraDist_; z<=dynamicObj.z+widthZ/2+this->cleanExtraDist_; z+=this->mapRes_){						
					
					Eigen::Vector3d point(x,y,z);
					if (this->isOccupied(point)) {
						// clean dynamic obj in static map
						Eigen::Vector3i idx;
						this->posToIndex(point, idx);
						int address = this->indexToAddress(idx);

						if (address<0 || address>=int(this->occupancy_.size())) {
							continue;
						}
						this->occupancy_[address] = this->pMinLog_;
						if (this->occAddressBeforeClean_.find(address)==this->occAddressBeforeClean_.end())
						{
							this->occAddressBeforeClean_.insert(address);
						}
					}
				}
			}
		}
	}


	void dynamicMap::mapRefine(){

		// get boxes
		this->rawBox3Ds_.clear();
		this->box2Ds_.clear();
		this->boxBirds_.clear();
		std::vector<box3D> rawBox3DsTemp;
		std::vector<cv::Rect> box2DsTemp;
		std::vector<cv::Rect> boxBirdsTemp;

		this->detector_->getBox3Ds(rawBox3DsTemp);
		this->detector_->getBox2Ds(box2DsTemp);
		this->detector_->getBoxBirds(boxBirdsTemp);

		if (rawBox3DsTemp.size()==0){
			this->getEmpty_ = true;
		}
		else {
			this->getEmpty_ = false;
		}

		// filter out unreasonable boxes from UV
		for (size_t i=0 ; i<rawBox3DsTemp.size() ; i++) {
			if ((rawBox3DsTemp[i].x_width>this->rawBoxWidthLimitUpper_ && rawBox3DsTemp[i].y_width>this->rawBoxWidthLimitUpper_ )) {
				continue;
			}
			if (rawBox3DsTemp[i].z_width>this->rawBoxHeightLimitUpper_) {
				rawBox3DsTemp[i].z_width = this->rawBoxHeightLimitUpper_-0.1;
			}

			this->rawBox3Ds_.push_back(rawBox3DsTemp[i]);
			this->box2Ds_.push_back(box2DsTemp[i]);
			this->boxBirds_.push_back(boxBirdsTemp[i]);
		}
		
		std::vector<box3D> box3dsTemp;

		// only search for points in the range of box(scalex,scaley,scalez) and that is occupied
		for (size_t i=0; i<this->rawBox3Ds_.size(); i++) {
			float xmax = -std::numeric_limits<float>::max();
			float ymax = -std::numeric_limits<float>::max();
			float zmax = -std::numeric_limits<float>::max();
			float xmin = std::numeric_limits<float>::max();
			float ymin = std::numeric_limits<float>::max();
			float zmin = std::numeric_limits<float>::max();
			if(this->detectionDebug_){
				cout << this->hint_  << ": 3D bounding boxes on the world frame: id: " << i << ", pos: (" << this->rawBox3Ds_[i].x << ", " << this->rawBox3Ds_[i].y << ", " << this->rawBox3Ds_[i].z << "), " << "size: (" 
				<< this->rawBox3Ds_[i].x_width << ", " << this->rawBox3Ds_[i].y_width << ", " << this->rawBox3Ds_[i].z_width << ")."<<  endl; 
			}
			
			float widthX = std::max(this->rawBox3Ds_[i].x_width,this->rawBox3Ds_[i].y_width)*this->mapRefineInflateCoef_;
			float widthY = std::max(this->rawBox3Ds_[i].x_width,this->rawBox3Ds_[i].y_width)*this->mapRefineInflateCoef_;
			float widthZ = this->rawBox3Ds_[i].z_width * this->mapRefineInflateCoef_;
			
			// start search
			bool inLoop = false;
			for (float x=this->rawBox3Ds_[i].x-widthX/2; x<=this->rawBox3Ds_[i].x+widthX/2; x+=this->mapRes_){
				for (float y=this->rawBox3Ds_[i].y-widthY/2; y<=this->rawBox3Ds_[i].y+widthY/2; y+=this->mapRes_){
					for (float z=this->mapRefineFloor_; z<=this->rawBox3Ds_[i].z+widthZ/2; z+=this->mapRes_){
						Eigen::Vector3d point(x,y,z);
						Eigen::Vector3i idx;
						this->posToIndex(point, idx);
						int address = this->indexToAddress(idx);
		
						if (this->isInflatedOccupied(point) || this->occAddressBeforeClean_.find(address)!=this->occAddressBeforeClean_.end() ){
							inLoop = true;
							xmin = (x<xmin)?x:xmin;
							ymin = (y<ymin)?y:ymin;
							zmin = (z<zmin)?z:zmin;
							xmax = (x>xmax)?x:xmax;  
							ymax = (y>ymax)?y:ymax;
							zmax = (z>zmax)?z:zmax;
						}
					}
				}
			}

			if (inLoop) {
				box3D box3dTemp;
				box3dTemp.x = (xmin+xmax)/2;
				box3dTemp.y = (ymin+ymax)/2;
				box3dTemp.z = this->rawBox3Ds_[i].z;
				box3dTemp.x_width = (xmax-xmin+ymax-ymin)/2.0;
				box3dTemp.y_width = (xmax-xmin+ymax-ymin)/2.0;
				box3dTemp.z_width = this->rawBox3Ds_[i].z_width;

				box3dsTemp.push_back(box3dTemp);

				if(this->detectionDebug_){
					cout << this->hint_ << ": Fused 3D bounding box " << i << " in the world frame: pos: (" << box3dTemp.x << ", " << box3dTemp.y << ", " << box3dTemp.z << "), " << "size: (" 
				<< box3dTemp.x_width << ", " << box3dTemp.y_width << ", " << box3dTemp.z_width << ")." <<  endl;
				}
			}
		}
		this->fusedBox3Ds_ = box3dsTemp;
	}


	void dynamicMap::trackAndIdentifyDynamic(){
		if (!this->getEmpty_){

			this->initTrack(this->fusedBox3Ds_, this->box2Ds_);

			this->trackAndEstimateVe(this->fusedBox3Ds_);

			this->identifyDynamicObjs(this->dynamicObjs_, this->dynamicHistInd_);
		}
	}


	void dynamicMap::initTrack(const std::vector<box3D> &nowBb, const std::vector<cv::Rect> &nowBb2d ) {
		// inherent the variables
		// Initate bounding boxes
		this->initTrackedStatesVar(this->preBb_, this->nowBb_, nowBb); // 3d bounding boxes
		this->initTrackedStatesVar(this->preOrg_, this->nowOrg_, nowBb); // original value copy of 3d bounding boxes
		this->initTrackedStatesVar(this->preBb2d_, this->nowBb2d_, nowBb2d); // 2d bounding boxes

		// Initiate tracking histories
		this->initTrackedStatesArr(this->preFix_, this->nowFix_, int(nowBb.size()));// fix size flag history   
		this->initTrackedStatesArr(this->preHist_, this->nowHist_, int(nowBb.size()));// Hist for all box
		this->initTrackedStatesArr(this->pre2dHist_, this->now2dHist_, int(nowBb.size()));// Hist for 2d box 
		this->initTrackedStatesArr(this->preVoteHist_, this->nowVoteHist_, int(nowBb.size()));// Voting Hist
		this->initTrackedStatesArr(this->preHistInFov_, this->nowHistInFov_, int(nowBb.size()));// Tracking Hist for boxes in FOV
		this->initTrackedStatesArr(this->preHistClean_, this->nowHistClean_, int(nowBb.size()));// Hist for dynamic region clean

		// Initiate other tracking states
		this->initTrackedStatesArr(this->preV_, this->nowV_, int(nowBb.size()));// queue of velocity prediction
		this->initTrackedStatesArr(this->fixCandidates_, this->preHistInFov_.size());// candidates box to be fixed
		this->initTrackedStatesArr(this->preDynamic_, this->nowDynamic_, int(nowBb.size()));// dynamic flag
		this->initTrackedStatesArr(this->preKfP_, this->nowKfP_, int(nowBb.size()));// kalman filters P
		this->initTrackedStatesArr(this->preKfStates_, this->nowKfStates_, int(nowBb.size()));// kalman filters states
		

		if (this->detectionDebug_){
			cout << this->hint_ << ": now bounding box frame size: " << nowBb.size() << endl;
			cout << this->hint_ << ": now history bounding box frame size: " << this->nowHistInFov_.size() << endl;
			cout << this->hint_ << ": previous bounding box frame size: " << this->preBb_.size() << endl;
		}

		// keep the size of history queues, store the sizes of boxes to be fixed
		this->storeFixCandSize();
		this->keepHistSize(this->pre2dHist_, this->nowHistSize_);
		this->keepHistSize(this->preHistClean_, this->preHistCleanSize_);
		
	}


	void dynamicMap::trackAndEstimateVe(std::vector<box3D> &dynamicObjs) {
		for(size_t nowId = 0; nowId < this->nowBb_.size(); nowId++)
		{
			// find best matched pairs candidates
			bool tracked = false;
			std::vector<int> checkQ;
			for(size_t preId = 0; preId < this->preBb_.size(); preId++)
			{
				if (this->checkTrack(this->nowOrg_[nowId], this->preOrg_[preId])) { 
					if (this->detectionDebug_){
						cout<< this->hint_ << ": Push "<< preId <<" as candidate." << endl;
					}
					
					checkQ.push_back(preId);
				}
			}
			int preId = 0;
			if (checkQ.size()>0){
				// for all best mateche pairs candidates, determine the best match
				tracked = true;
				int ind = 0;
				double minDist = std::numeric_limits<float>::max();
				if (checkQ.size()>1){
					
					for (size_t i=0 ; i<checkQ.size() ; i++) {
						double dist = distance(this->nowOrg_[nowId].x, this->nowOrg_[nowId].y, this->preOrg_[checkQ[i]].x, this->preOrg_[checkQ[i]].y);
						if (minDist > dist){
							minDist = dist;
							ind = checkQ[i];
						}
					}
					preId = ind;
				}
				else {
					preId = checkQ[0];
				}

				if (this->detectionDebug_){
					cout<< this->hint_ << ": Track Now ID: " << nowId << " from previous ID: " << preId << endl;
				}

				this->updateTrackedStates(nowId, preId);
			}
			
			// for new boxes
			if (!tracked) { 
				this->updateNonTrackedStates(nowId);
			}      
		}
	}  


	void dynamicMap::identifyDynamicObjs(std::vector<box3D> &dynamicObjs, std::vector<int> &dynamicHistInd) {
		std::vector<bool> isDynamic;
		isDynamic.clear();
		isDynamic.resize(this->nowBb_.size());

		this->firstLayerDetect(isDynamic);

		this->secondLayerDetect(isDynamic, dynamicObjs, dynamicHistInd);
	}


	void dynamicMap::storeFixCandSize(){
		// for each box, track and fix the size after showing up for certain frames
		for (size_t i=0 ; i<this->preHistInFov_.size() ; i++) {
			if (this->preHistInFov_[i].size() > this->nowHistSize_) {
				this->preHistInFov_[i].pop_front();

				if (!this->preFix_[i]) {
					this->preFix_[i] = true;
				}
			
				if (this->preHistInFov_[i].back().z_width<this->nowBb_[i].z_width){
					this->fixCandidates_[i].z_width = nowBb_[i].z_width;
				}
				else {
					this->fixCandidates_[i].z_width = this->preHistInFov_[i].back().z_width;
				}

				this->fixCandidates_[i].x_width = this->preHistInFov_[i].back().x_width;
				this->fixCandidates_[i].y_width = this->preHistInFov_[i].back().y_width;
			}
		}  
	}

	// keep the history queue size as "size"
	template <typename T> void dynamicMap::keepHistSize(T &HistContainer, int size){
		for (size_t i=0 ; i<HistContainer.size() ; i++) {
			if (int(HistContainer[i].size()) > size) {
				HistContainer[i].pop_front();
			}
		}
	}


	void dynamicMap::updateNonTrackedStates(const int &nowId){
		if (this->detectionDebug_){
			cout << this->hint_ << ": ID: " << nowId << "Not tracked!" << endl;
		}
		// initiate fix flag
		this->nowFix_[nowId] = false;
		this->nowDynamic_[nowId] = false;

		// init velocity estimate
		MatrixXd V(2,1);
		V << 0.0, 0.0;

		// add current 3D box to box history only if when it is full in the FOV.
		if (this->isInFov(this->box2Ds_[nowId])) {
			this->nowHistInFov_[nowId].push_back(this->nowBb_[nowId]);
		}
		this->now2dHist_[nowId].push_back(this->nowBb2d_[nowId]);
		this->nowHist_[nowId].push_back(this->nowBb_[nowId]);
		this->nowHistClean_[nowId].push_back(this->nowBb_[nowId]);

		// init kalman filter
		this->initKf(this->nowKfStates_[nowId], this->nowKfP_[nowId], this->nowBb_[nowId]);
	}


	void dynamicMap::updateTrackedStates(const int &nowId, const int &preId){
		// update fix flag
		this->nowFix_[nowId] = this->preFix_[preId];
		this->nowDynamic_[nowId] = this->preDynamic_[preId];

		// inherit box Hist
		this->nowHistInFov_[nowId] = this->preHistInFov_[preId];
		this->now2dHist_[nowId] = this->pre2dHist_[preId];
		this->nowHist_[nowId] = this->preHist_[preId];
		this->nowHistClean_[nowId] = this->preHistClean_[preId];

		if (this->nowFix_[nowId]) {
			this->nowBb_[nowId].x_width = this->fixCandidates_[preId].x_width;
			this->nowBb_[nowId].y_width = this->fixCandidates_[preId].y_width;
			this->nowBb_[nowId].z_width = this->fixCandidates_[preId].z_width;
		}

		// add current 3D box to box history only if when it is full in the FOV.
		if (this->isInFov(this->box2Ds_[nowId])) {
			this->nowHistInFov_[nowId].push_back(this->nowBb_[nowId]);// tracked box will have fixed size
		}
		this->now2dHist_[nowId].push_back(this->nowBb2d_[nowId]);
		this->nowHist_[nowId].push_back(this->nowBb_[nowId]);
		this->nowHistClean_[nowId].push_back(this->nowBb_[nowId]);

		// inherit vote Hist
		this->nowVoteHist_[nowId] = this->preVoteHist_[preId];

		// inherit velocity prediction  
		this->nowV_[nowId] = this->preV_[preId];

		MatrixXd z(4,1); // measurement
		// calculate velocity of box centor from pre to now as the measrued velocity
		double z_vx = (this->nowBb_[nowId].x - this->preBb_[preId].x)/this->ts_;
		double z_vy = (this->nowBb_[nowId].y - this->preBb_[preId].y)/this->ts_;
		double z_cx = this->nowBb_[nowId].x;
		double z_cy = this->nowBb_[nowId].y;
		z << z_cx, z_cy, z_vx, z_vy;

		// run the Kalman Filter 
		this->kfEstimate( this->nowKfStates_[nowId], this->nowKfP_[nowId], z, this->preKfStates_[preId], this->preKfP_[preId]);   
		MatrixXd V(2,1);

		V << this->nowKfStates_[nowId](2,0),this->nowKfStates_[nowId](3,0);
		if (norm(V(0,0),V(1,0)) > 0.2){
			cv::Rect pre;
			if (this->pre2dHist_[preId].size()){
				pre = this->pre2dHist_[preId][0];
			}
			else {
				pre = this->preBb2d_[preId];
			}
			
			if ((abs(this->nowBb2d_[nowId].tl().x-pre.tl().x) <3 || abs(this->nowBb2d_[nowId].br().x-pre.br().x )< 3) && this->nowBb2d_[nowId].br().x<this->fovXMarginUpper_ && this->nowBb2d_[nowId].tl().x>this->fovXMarginLower_)
			{
				if (this->detectionDebug_||this->veDebug_){
					cout << this->hint_ << "Velocity Cancel." << endl << endl;
					cout << this->hint_ << ": " << this->nowBb2d_[nowId].tl().x << " " << this->preBb2d_[preId].tl().x << " " << this->nowBb2d_[nowId].br().x << " " << this->preBb2d_[preId].br().x << endl;
				}
				V(0,0) = 0.01;
				V(1,0) = 0.01;
			}
		}
		this->nowV_[nowId].push_back(V);
		this->nowBb_[nowId].x = this->nowKfStates_[nowId](0,0);
		this->nowBb_[nowId].y = this->nowKfStates_[nowId](1,0);
		if (this->veDebug_){
			cout << this->hint_ << ": Estimated velocity, ID: " << nowId << ", V: " << V(0,0) << ", " << V(1,0) << endl;
		}
	}


	void dynamicMap::firstLayerDetect(std::vector<bool> &isDynamic){
		// calculate running average as final velocity estimation
		for (size_t i=0 ; i<this->nowV_.size() ; i++) {
			isDynamic[i] = false;
			if (this->nowV_[i].size() > this->avgVHistSize_) {
				this->nowV_[i].pop_front();
			}

			if (this->nowHist_[i].size() > this->nowHistSize_) {
				this->nowHist_[i].pop_front();
			}
			double sumVx = 0.0;
			double sumVy = 0.0;
			for (size_t j=0 ; j<this->nowV_[i].size() ; j++) {
				sumVx += this->nowV_[i][j](0,0);  
				sumVy += this->nowV_[i][j](1,0);   
			}

			// take average of esetimated velocity
			double Vx = (sumVx/this->nowV_[i].size());
			double Vy = (sumVy/this->nowV_[i].size());


			// continuity filter 
			double contAvg = this->continuityFilter(i);
			
			this->nowBb_[i].Vx = Vx;
			this->nowBb_[i].Vy = Vy; 


			if (this->detectionDebug_){
				cout << this->hint_ << ": Object ID: " << i << endl;
				cout << this->hint_ << ": Average Vel: " << norm(Vx, Vy) << ", queue length: " << this->nowHist_[i].size() << ", continuity: " << contAvg << endl;
				cout << this->hint_ << ": Pos:  (" << this->nowBb_[i].x << ", " << this->nowBb_[i].y << ", " << this->nowBb_[i].z << ")" << ", Size:  (" << this->nowBb_[i].x_width << ", " << this->nowBb_[i].y_width << ", " << this->nowBb_[i].z_width << ")." << endl;
				cout << this->hint_ << ": Size Ratio: " << this->nowBb_[i].x_width/this->nowBb_[i].z_width << " " << this->nowBb_[i].y_width/this->nowBb_[i].z_width << endl;
			}

			if(this->nowDynamic_[i]) {
				this->nowVoteHist_[i].push_back(1);
				isDynamic[i] = true;
			}
			else if (norm(Vx,Vy) > this->vAvgEstimatedMetric_ &&  contAvg>this->contTresh_) {
				
				this->nowVoteHist_[i].push_back(1);
				isDynamic[i] = true;
				double length = std::max(this->nowBb_[i].x_width,this->nowBb_[i].y_width);
				double width = std::min(this->nowBb_[i].x_width,this->nowBb_[i].y_width);
				double height = this->nowBb_[i].z_width;
				if (length/height>this->sizeRatioUpper_ || length/height<this->sizeRatioLower_ || width/height<this->sizeRatioLower_ || width/height>this->sizeRatioUpper_
				|| (this->nowBb_[i].x_width<this->fusedBoxWidthLimitLower_ && this->nowBb_[i].y_width<this->fusedBoxWidthLimitLower_)
				|| (this->nowBb_[i].x_width>this->fusedBoxWidthLimitUpper_ && this->nowBb_[i].y_width>this->fusedBoxWidthLimitUpper_) 
				|| this->nowBb_[i].z_width<this->fusedBoxHeightLimitLower_ || this->nowBb_[i].z_width>this->fusedBoxHeightLimitUpper_
				|| (this->nowBb_[i].z-this->nowBb_[i].z_width/2 > this->lowestPointLimit_)) {
					if (this->detectionDebug_){     
						if ((length/height>this->sizeRatioUpper_ )|| (length/height<this->sizeRatioLower_ )|| (width/height<this->sizeRatioLower_) || (width/height>this->sizeRatioUpper_)){
							cout << this->hint_ <<  "1: " << " " << endl;
						}
						if ((this->nowBb_[i].x_width<this->fusedBoxWidthLimitLower_) && (this->nowBb_[i].y_width<this->fusedBoxWidthLimitLower_)){
							cout << this->hint_ <<  "2: " << " " << endl;
						}
						if ((this->nowBb_[i].x_width>this->fusedBoxWidthLimitUpper_) && (this->nowBb_[i].y_width>this->fusedBoxWidthLimitUpper_)){
							cout << this->hint_ <<  "3: " << " " << endl;
							cout << this->hint_ << ": " << this->nowBb_[i].x_width << " " << this->nowBb_[i].y_width << " " << this->fusedBoxWidthLimitUpper_ << " " <<fusedBoxWidthLimitUpper_ << endl;
						}
						if ((this->nowBb_[i].z_width<this->fusedBoxHeightLimitLower_) || (this->nowBb_[i].z_width>this->fusedBoxHeightLimitUpper_)|| ((this->nowBb_[i].z-this->nowBb_[i].z_width/2) > this->lowestPointLimit_)){
							cout << this->hint_ <<  "4: " << " " << endl;
							cout << this->hint_ << ": " << this->nowBb_[i].z_width << " " << this->fusedBoxHeightLimitUpper_ << " " << this->nowBb_[i].z << " " << this->lowestPointLimit_ << endl;
						}
						std::cout << this->hint_ << ": First Layer Fail!" << endl;
						std::cout << this->hint_ << ": Size wrong, ID: " << i << ", proc count: " << this->nowVoteHist_[i].back() << ", is dynamic? " << isDynamic[i] << endl;  
					}
					
					this->nowVoteHist_[i].back() = 0;
					isDynamic[i] = false;
				
				}
				else {
					if (this->detectionDebug_){
						std::cout << this->hint_ << ": First Layer pass!" << endl;
					}
				}

			}   
			else {
				if (this->detectionDebug_){
						std::cout << this->hint_ << ": First Layer Fail!" << endl;
				}
				
				this->nowVoteHist_[i].push_back(0);
				isDynamic[i] = false;
			}   
    
		}  
   
	}


	void dynamicMap::secondLayerDetect(std::vector<bool> &isDynamic,std::vector<box3D> &dynamicObjs, std::vector<int> &dynamicHistInd){

		std::vector<int> preDynamicHistInd = dynamicHistInd;
		
		dynamicObjs.clear();
		dynamicHistInd.clear();
		for (size_t i=0 ; i<this->nowVoteHist_.size() ; i++) {			
			if (this->nowVoteHist_[i].size() > this->voteHistSize_) {
				this->nowVoteHist_[i].pop_front();
			}  

			// total votes
			int sum = 0;
			for (size_t j=0 ; j<this->nowVoteHist_[i].size() ; j++) {
				sum += this->nowVoteHist_[i][j];
			}  
			if (this->detectionDebug_){
				cout << this->hint_ << ": ID " << i << ", sum: " << sum << endl;  
				cout << this->hint_ << ": Vote for now, ID: " << i << ", " << this->nowVoteHist_[i].back() << endl;
			}
			
			// vote for detemining if the box is dynamic
			if (this->nowDynamic_[i] || sum >= this->dynamicThresh_) {
				if(sum == int(this->voteHistSize_)) {    
					this->nowDynamic_[i] = true;
				}
				isDynamic[i] = true;
			}
			else if (sum <=this->staticThresh_ ) {
	
				isDynamic[i] = false;
			}
			else {
				isDynamic[i] = this->nowVoteHist_[i].back()==1;
			}

			// push dynamic obstacles
			if (isDynamic[i]) {				
				dynamicObjs.push_back(nowBb_[i]);
				dynamicObjs.back().z = dynamicObjs.back().z_width/2+this->groundThick_;
				dynamicHistInd.push_back(i);
				if (this->detectionDebug_){
					cout << this->hint_ << ": Dynamic obstacle: ID: " << i << ", Size:  (" << dynamicObjs.back().x_width << ", " << dynamicObjs.back().y_width << ", " << dynamicObjs.back().z_width << ")." << endl;
				}
			}
			else {
				if (this->detectionDebug_){
					cout << this->hint_ << ": Static obstacle ID: " << i << endl;
				}
			}
		}
	}


	double dynamicMap::continuityFilter(const int &nowId) {
		int countShake = 0;
		double contAvg = this->contTresh_;
		if (this->detectionDebug_){
			cout << this->hint_ << ": History size: " << this->nowHist_[nowId].size() << ", continuity size: " << this->CFHistSize_ << endl;
		}

		if (this->nowHist_[nowId].size()>=this->CFHistSize_) {
			std::vector<double> angles;   
			angles.clear();
			double angleSum = 0;
			double vx = 0;
			double vy = 0;
			double preVx = 0;
			double preVy = 0;

			for (size_t j=this->CFHistSize_-this->CFInterval_ ; j<this->CFHistSize_ ; j++) {
				vx = this->nowHist_[nowId][j].x-this->nowHist_[nowId][j-(this->CFHistSize_-this->CFInterval_)].x;
				vy = this->nowHist_[nowId][j].y-this->nowHist_[nowId][j-(this->CFHistSize_-this->CFInterval_)].y;

				if (j>this->CFHistSize_-this->CFInterval_) {
					double angle = (vx*preVx+vy*preVy)/(norm(vx, vy)+norm(preVx, preVy));
					angles.push_back(angle);
					countShake += angle>0;
					if(this->detectionDebug_){
						cout << this->hint_ << ": Vx: " << vx << ", Vy: " << vy << ", Angle: " << angle << endl;
					}
					angleSum += angle;
				}
				preVx = vx;
				preVy = vy;  
			}
			contAvg = angleSum/(this->CFInterval_-1);
		}

		return contAvg;
	}


	void dynamicMap::initKf(Eigen::MatrixXd &statesNowId, Eigen::MatrixXd &pNowId, const box3D &boxNowId) {
		// initialize filter 
		Eigen::MatrixXd A(4, 4);
		A <<    1, 0, this->ts_, 0,
				0, 1, 0,        this->ts_,
				0, 0, 1,        0,
				0, 0, 0,        1;
		Eigen::MatrixXd H(4,4);
		H <<    1, 0, 0, 0,
				0, 1, 0, 0,
				0, 0, 1, 0,
				0, 0, 0, 1;
		Eigen::MatrixXd P = MatrixXd::Identity(4, 4) * this->eP_;
		Eigen::MatrixXd Q = MatrixXd::Identity(4, 4) * this->eQ_;
		Eigen::MatrixXd R = MatrixXd::Identity(4, 4) * this->eR_;
		this->A_ = A;
		this->H_ = H;
		this->Q_ = Q;
		this->R_ = R;
		Eigen::MatrixXd states(4,1);
		states << boxNowId.x, boxNowId.y, 0, 0;
		statesNowId = states;
		pNowId = P;
	}

	void dynamicMap::kfEstimate(Eigen::MatrixXd &UpdateStates, Eigen::MatrixXd &UpdateP, const Eigen::MatrixXd &z, const Eigen::MatrixXd &states, const Eigen::MatrixXd &P) {
		// predict
		UpdateStates = this->A_ * states ;;
		UpdateP = this->A_ * P * this->A_.transpose() + this->Q_;
		// update
		Eigen::MatrixXd S = this->R_ + this->H_ * UpdateP * this->H_.transpose(); // innovation matrix
		Eigen::MatrixXd K = UpdateP * this->H_.transpose() * S.inverse(); // kalman gain
		UpdateStates = UpdateStates + K * (z - this->H_ * UpdateStates);
		UpdateP = (MatrixXd::Identity(UpdateP.rows(),UpdateP.cols()) - K * this->H_) * UpdateP;
	}

	void dynamicMap::predictTrajectory() {
		this->trajs_.clear();
		this->bestTrajs_.clear();
		this->endPoints_.clear();
		this->generateTraj(this->dynamicObjs_);
		
		// check collision
		this->checkCollisionPos();
		this->readBoxInfoMarkov(this->dynamicObjs_,this->endPoints_);
		this->predict();
		for (size_t i=0 ; i<this->trajs_.size() ; i++) {
			this->bestTrajs_[i] = this->trajs_[i][this->resultInd_[i]];
			this->bestTrajs_[i].pop_back();
		}
	}

	void dynamicMap::generateTraj(const std::vector<box3D> &objs){

		this->trajs_.clear();
		this->trajsForColliCheck_.clear();
		this->bestTrajs_.clear();
		this->endPoints_.clear();
		
		if (objs.size()>0) {
			this->trajs_.resize(objs.size());
			this->trajsForColliCheck_.resize(objs.size());
			this->bestTrajs_.resize(objs.size());
			this->endPoints_.resize(objs.size());

			for (size_t i=0 ; i<objs.size() ; i++){
				this->trajs_[i].resize(this->n_);
				this->trajsForColliCheck_[i].resize(this->n_);
				this->endPoints_[i].resize(this->n_);

				// calc transformation coefficients
				// cos  sin,-tx      x      
				// -sin cos -ty   *  y 
				// 0    0    1       1
				double vx = objs[i].Vx;
				double vy = objs[i].Vy;
				double v = norm(vx, vy);
				double cos = vy/v;
				double sin = vx/v;
				double tx = -1*double(objs[i].x);
				double ty = -1*double(objs[i].y);

				
				// generate trajectories
				// go straight
				double x = 0.0; 
				double y = 0.0;
				double tColliCheck = (v<0.1)?this->t_:this->colliCheckEndDist_/v;
				v = (v<0.01)?0.01:v;
				geometry_msgs::Point p;
				p.z = objs[i].z;
				int numPoints = int(tColliCheck/this->ts_);
				for (int j=0 ; j<numPoints; j++){
					y += v*this->ts_;
					p.x = cos*x + sin*y - tx; 
					p.y = -sin*x +cos*y - ty; // transform
					p.z = p.z;
					this->trajs_[i][2].push_back(p);
					this->trajsForColliCheck_[i][2].push_back(p);
				}


				// other curves
				for (size_t k=0 ; k<this->coefs_.size() ; k++) {
					
					x = 0.0; y = 0.0; p.z = objs[i].z;
					p.x = cos*x + sin*y -tx; p.y = -sin*x +cos*y - ty; // transform
					this->trajs_[i][k+1].push_back(p);
					this->trajsForColliCheck_[i][k+1].push_back(p);
					this->trajs_[i][k+1+this->coefs_.size()].push_back(p);
					this->trajsForColliCheck_[i][k+1+this->coefs_.size()].push_back(p);

					for (int j=0 ; j<numPoints ; j++){
						double tanTraj = 6*coefs_[k](0,0)*std::pow(x,5) + 5*coefs_[k](1,0)*std::pow(x,4) + 4*coefs_[k](2,0)*std::pow(x,3) + 3*coefs_[k](3,0)*std::pow(x,2) + 2*coefs_[k](4,0)*std::pow(x,1) + coefs_[k](5,0);
						double cosTraj = 1/norm(tanTraj,1.0);
						double sinTraj = tanTraj/norm(tanTraj,1.0);
						x += v*this->ts_*cosTraj;
						y += v*this->ts_*sinTraj;
						p.x = cos*x + sin*y - tx; 
						p.y = -sin*x +cos*y - ty; // transform
						p.z = p.z;

						this->trajs_[i][2*this->coefs_.size()-k].push_back(p);
						this->trajsForColliCheck_[i][2*this->coefs_.size()-k].push_back(p);


						// symetric curve on the other side
						x = -x;
						p.x = cos*x + sin*y - tx; 
						p.y = -sin*x +cos*y - ty; // transform

						this->trajs_[i][k].push_back(p);
						this->trajsForColliCheck_[i][k].push_back(p);
						x = -x;// x back to org value
					}       
				}
			}
		}
			
	}


	void dynamicMap::readBoxInfoMarkov(const std::vector<box3D> &nowBbMarkov, const std::vector<std::vector<int>> &endPoints){
		this->preBbMarkov_= this->nowBbMarkov_;
    	this->nowBbMarkov_ = nowBbMarkov;
		this->PPre_ = this->PNow_;
		this->PS_.clear();
		this->PS_.resize(this->nowBbMarkov_.size());
		this->PNow_.clear();
		this->PNow_.resize(this->nowBbMarkov_.size());

		this->resultInd_.clear();
		this->resultInd_.resize(this->nowBbMarkov_.size());
		MatrixXd P_S(1,5);
		P_S << 	0, 0, 0, 0, 0;
		// softmax
		for (size_t i=0 ; i<nowBbMarkov_.size() ; i++){
			for (size_t j=0 ; j<endPoints[i].size() ; j++) {
				P_S(0,j) = 0.1*float(endPoints[i][j]);
				P_S(0,j) = exp(P_S(0,j));
			}  
			double exp_sum = P_S.sum();
			P_S = P_S/exp_sum;

			this->PS_[i] = P_S;
		}
	}


	void dynamicMap::predict() {
		for(size_t nowId = 0; nowId < this->nowBbMarkov_.size(); nowId++)
		{
			bool tracked = false;
			for(size_t preId = 0; preId < this->preBbMarkov_.size(); preId++) // update markov chain
			{  
				if (this->checkTrack(this->nowBbMarkov_[nowId],this->preBbMarkov_[preId])){
					tracked = true;   
					this->PNow_[nowId] = this->PPre_[preId]*this->PT_;

					for (int i=0 ; i<this->PNow_[nowId].cols() ; i++){
						this->PNow_[nowId](0,i) = this->PNow_[nowId](0,i)*this->PS_[nowId](0,i);
					}

					this->PNow_[nowId] = this->PNow_[nowId]/this->PNow_[nowId].sum();
				}   
			}
			if (!tracked) { // init markov chain
				this->PNow_[nowId] = this->PI_;
				for (int i=0 ; i<this->PNow_[nowId].cols() ; i++){
					this->PNow_[nowId](0,i) = this->PNow_[nowId](0,i)*this->PS_[nowId](0,i);
				}
			}  

			// choose best traj
			this->resultInd_[nowId] = 0;   
			double prob = 0;
			for (int i=0; i<this->PNow_[nowId].size() ; i++){
				if(this->PNow_[nowId](0,i)>prob){
					prob = this->PNow_[nowId](0,i);
					this->resultInd_[nowId] = i;
				}   
			}
		}
	}


	void dynamicMap::checkCollisionPos(){
		// check collision
		double x, y, z;  
		for (size_t i=0 ; i<this->trajsForColliCheck_.size() ; i++) { //each obj
			for (size_t j=0 ; j<this->trajsForColliCheck_[i].size() ; j++) { // each traj
				bool check_on = false;
				for (size_t k=0 ; k<this->trajsForColliCheck_[i][j].size() ; k++) { // each point
					
					x = this->trajsForColliCheck_[i][j][k].x;
					y = this->trajsForColliCheck_[i][j][k].y;
					z = this->trajsForColliCheck_[i][j][k].z;
 
					this->endPoints_[i][j] = k;
					Eigen::Vector3d point(x,y,z);
					if (this->isInflatedOccupied(point) || this->isOccupied(point)) {
						double collisionDist = distance(x, y, double(this->dynamicObjs_[i].x), double(this->dynamicObjs_[i].y));
						if (check_on || collisionDist>=this->colliCheckStartDist_) {
							break;
						}
					}
					else {
						check_on = true;
					}
				}
			}
		}
	}

	// toolbox functions
	bool dynamicMap::checkTrack(const box3D &now, const box3D &pre) {

		float overlapX = std::min( now.y+now.y_width/2-(pre.y-pre.y_width/2) , pre.y+pre.y_width/2-(now.y-now.y_width/2) );
		float overlapY = std::min( now.x+now.x_width/2-(pre.x-pre.x_width/2) , pre.x+pre.x_width/2-(now.x-now.x_width/2) );
		double dist = distance(now.x, now.y, pre.x, pre.y);
		double metric = 0.22;
		bool overlap = overlapX*overlapY/(now.x_width*now.y_width) > this->overlapThreshold_ && overlapY*overlapY/(pre.x_width*pre.y_width) > this->overlapThreshold_;

		return ((overlapX>0 && overlapY>0) && overlap) || dist < metric;
	}      


	void dynamicMap::getPredTraj(std::vector<std::vector<std::vector<geometry_msgs::Point>>>& predTraj){
		predTraj = this->trajs_;
	}


	void dynamicMap::getObstaclesPos(std::vector<Eigen::Vector3d> &obstaclesPos) {
		obstaclesPos.clear();
		Eigen::Vector3d posVect(3);
		for (size_t i=0 ; i<this->dynamicObjs_.size() ; i++) {
			posVect(0) = this->dynamicObjs_[i].x;
			posVect(1) = this->dynamicObjs_[i].y;
			posVect(2) = this->dynamicObjs_[i].z-this->dynamicObjs_[i].z_width/2;
			obstaclesPos.push_back(posVect);
		}
	}


	void dynamicMap::getObstaclesVel(std::vector<Eigen::Vector3d> &obstaclesVel){
		obstaclesVel.clear();
		Eigen::Vector3d velVect(3);
		for (size_t i=0 ; i<this->dynamicObjs_.size() ; i++) {
			velVect(0) = this->dynamicObjs_[i].Vx;
			velVect(1) = this->dynamicObjs_[i].Vy;
			velVect(2) = 0.0;
			obstaclesVel.push_back(velVect);
		}
	}

	void dynamicMap::getObstaclesSize(std::vector<Eigen::Vector3d> &obstaclesSize) {
		obstaclesSize.clear();
		Eigen::Vector3d sizeVect(3);
		for (size_t i=0 ; i<this->dynamicObjs_.size() ; i++) {
			sizeVect(0) = this->dynamicObjs_[i].x_width;
			sizeVect(1) = this->dynamicObjs_[i].y_width;
			sizeVect(2) = this->dynamicObjs_[i].z_width;
			obstaclesSize.push_back(sizeVect);
		}
	}

	void dynamicMap::getDynamicObstacles(std::vector<Eigen::Vector3d> &obstaclesPos, std::vector<Eigen::Vector3d> &obstaclesVel, std::vector<Eigen::Vector3d> &obstaclesSize){
		this->getObstaclesPos(obstaclesPos);
		this->getObstaclesVel(obstaclesVel);
		this->getObstaclesSize(obstaclesSize);
	}

	void dynamicMap::publishVelAndPos(const std::vector<box3D> &dynamicObjs){
		std_msgs::Float64 velocity;
		geometry_msgs::PointStamped p;
		for (size_t i=0 ; i<dynamicObjs.size() ; i++){
			double v = norm(dynamicObjs[i].Vx, dynamicObjs[i].Vy);
			velocity.data = v;
			p.header.frame_id = "map";
			p.header.stamp = ros::Time::now();
			p.point.x = dynamicObjs[i].x;
			p.point.y = dynamicObjs[i].y;
			p.point.z = dynamicObjs[i].z;
			this->dynamicVelPub_.publish(velocity);
			this->dynamicPosPub_.publish(p);
		}
	}


	void dynamicMap::publish3dBox(const std::vector<box3D> &boxes, const ros::Publisher &publisher, const char &color) {

		// visualization using bounding boxes 
		visualization_msgs::Marker line;
		visualization_msgs::MarkerArray lines;
		line.header.frame_id = "map";
		line.type = visualization_msgs::Marker::LINE_LIST;
		line.action = visualization_msgs::Marker::ADD;
		line.ns = "box3D";	
		line.scale.x = 0.1;

		if (color=='g') {
			line.color.g = 1.0;
		}
		else if (color=='b') {
			line.color.b = 1.0;
		}
		else {
			line.color.r = 1.0;
		}

		line.color.a = 1.0;
		line.lifetime = ros::Duration(0.1);

		for(size_t i = 0; i < boxes.size(); i++){
			// visualization msgs

			double x = boxes[i].x; 
			double y = boxes[i].y; 
			double z = boxes[i].z; 

			double x_width = std::max(boxes[i].x_width,boxes[i].y_width);
			double y_width = std::max(boxes[i].x_width,boxes[i].y_width);
			double z_width = boxes[i].z_width;
			
			vector<geometry_msgs::Point> verts;
			geometry_msgs::Point p;
			// vertice 0
			p.x = x-x_width / 2.; p.y = y-y_width / 2.; p.z = z-z_width / 2.;
			verts.push_back(p);

			// vertice 1
			p.x = x-x_width / 2.; p.y = y+y_width / 2.; p.z = z-z_width / 2.;
			verts.push_back(p);

			// vertice 2
			p.x = x+x_width / 2.; p.y = y+y_width / 2.; p.z = z-z_width / 2.;
			verts.push_back(p);

			// vertice 3
			p.x = x+x_width / 2.; p.y = y-y_width / 2.; p.z = z-z_width / 2.;
			verts.push_back(p);

			// vertice 4
			p.x = x-x_width / 2.; p.y = y-y_width / 2.; p.z = z+z_width / 2.;
			verts.push_back(p);

			// vertice 5
			p.x = x-x_width / 2.; p.y = y+y_width / 2.; p.z = z+z_width / 2.;
			verts.push_back(p);

			// vertice 6
			p.x = x+x_width / 2.; p.y = y+y_width / 2.; p.z = z+z_width / 2.;
			verts.push_back(p);

			// vertice 7
			p.x = x+x_width / 2.; p.y = y-y_width / 2.; p.z = z+z_width / 2.;
			verts.push_back(p);
			
			int vert_idx[12][2] = {
				{0,1},
				{1,2},
				{2,3},
				{0,3},
				{0,4},
				{1,5},
				{3,7},
				{2,6},
				{4,5},
				{5,6},
				{4,7},
				{6,7}
			};
			
			for (size_t i=0;i<12;i++){
				line.points.push_back(verts[vert_idx[i][0]]);
				line.points.push_back(verts[vert_idx[i][1]]);
			}
			
			lines.markers.push_back(line);
			
			line.id++;
		}
		// publish
		publisher.publish(lines);
	}


	void dynamicMap::publishAllTraj(){

		visualization_msgs::Marker line; 
		visualization_msgs::MarkerArray lines;   
		line.header.frame_id = "map";
		line.type = visualization_msgs::Marker::LINE_LIST;
		line.action = visualization_msgs::Marker::ADD;
		line.scale.x = 0.05;
		line.color.b = 1.0;
		line.color.a = 1.0;
		line.lifetime = ros::Duration(0.1);

		for (size_t i=0 ; i<this->dynamicObjs_.size() ; i++) {
			// Line list is 
			
			for (size_t j=0 ; j<this->trajs_[i].size() ; j++) {
				if (this->endPoints_[i][j]<this->t_/this->ts_-3) { // don't show collision trajs
					continue;
				}
				if (this->trajs_[i][j].size()<=1) {
					continue;
				}
				for (size_t k=0; k<this->trajs_[i][j].size()-2 ; k++) {
					line.points.push_back(this->trajs_[i][j][k]);
					line.points.push_back(this->trajs_[i][j][k+1]);
				}  
			}
			  
			lines.markers.push_back(line);
			line.id++;
		}
		this->obstacleTrajPub_.publish(lines);
	}


	void dynamicMap::publishBestTraj(){
		visualization_msgs::Marker line; 
		visualization_msgs::MarkerArray lines;   
		line.header.frame_id = "map";
		line.type = visualization_msgs::Marker::LINE_LIST;
		line.action = visualization_msgs::Marker::ADD;
		   
		line.scale.x = 0.05;
		
		line.color.g = 1.0;
		line.color.a = 1.0;
		line.lifetime = ros::Duration(0.1);
		for (size_t i=0 ; i<this->dynamicObjs_.size() ; i++) {
			// Line list is 
			if (this->bestTrajs_[i].size()<=1) {
				continue;  
			}   
			for (size_t k=0; k<this->bestTrajs_[i].size()-1 ; k++) {
				line.points.push_back(this->bestTrajs_[i][k]);
				line.points.push_back(this->bestTrajs_[i][k+1]);
			}
			
			lines.markers.push_back(line); 
			line.id++;
		}
		this->obstacleTrajPub_.publish(lines);
	}
}

