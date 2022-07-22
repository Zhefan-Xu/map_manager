/*
	FILE: occupancyMap.cpp
	--------------------------------------
	function definition of occupancy map
*/
#include <map_manager/occupancyMap.h>

namespace mapManager{
	occMap::occMap(){
		this->ns_ = "occupancy_map";
		this->hint_ = "[OccMap]";
	}

	void occMap::initMap(const ros::NodeHandle& nh){
		this->nh_ = nh;
		this->initParam();
		this->registerPub();
		this->registerCallback();
	}

	void occMap::initParam(){
		// localization mode
		if (not this->nh_.getParam(this->ns_ + "/localization_mode", this->localizationMode_)){
			this->localizationMode_ = 0;
			cout << this->hint_ << ": No localization mode option. Use default: pose" << endl;
		}
		else{
			cout << this->hint_ << ": Localizaiton mode: pose (0)/odom (1). Your option: : " << this->localizationMode_ << endl;
		}

		// depth topic name
		if (not this->nh_.getParam(this->ns_ + "/depth_image_topic", this->depthTopicName_)){
			this->depthTopicName_ = "/camera/depth/image_raw";
			cout << this->hint_ << ": No depth image topic name. Use default: /camera/depth/image_raw" << endl;
		}
		else{
			cout << this->hint_ << ": Depth topic: " << this->depthTopicName_ << endl;
		}

		if (this->localizationMode_ == 0){
			// odom topic name
			if (not this->nh_.getParam(this->ns_ + "/pose_topic", this->poseTopicName_)){
				this->poseTopicName_ = "/CERLAB/quadcopter/pose";
				cout << this->hint_ << ": No pose topic name. Use default: /CERLAB/quadcopter/pose" << endl;
			}
			else{
				cout << this->hint_ << ": Pose topic: " << this->poseTopicName_ << endl;
			}			
		}

		if (this->localizationMode_ == 1){
			// pose topic name
			if (not this->nh_.getParam(this->ns_ + "/odom_topic", this->odomTopicName_)){
				this->odomTopicName_ = "/CERLAB/quadcopter/odom";
				cout << this->hint_ << ": No odom topic name. Use default: /CERLAB/quadcopter/odom" << endl;
			}
			else{
				cout << this->hint_ << ": Odom topic: " << this->odomTopicName_ << endl;
			}
		}

		std::vector<double> robotSizeVec (3);
		if (not this->nh_.getParam(this->ns_ + "/robot_size", robotSizeVec)){
			robotSizeVec = std::vector<double>{0.5, 0.5, 0.3};
		}
		else{
			cout << this->hint_ << ": robot size: " << "[" << robotSizeVec[0]  << ", " << robotSizeVec[1] << ", "<< robotSizeVec[2] << "]" << endl;
		}
		this->robotSize_(0) = robotSizeVec[0]; this->robotSize_(1) = robotSizeVec[1]; this->robotSize_(2) = robotSizeVec[2];

		std::vector<double> depthIntrinsics (4);
		if (not this->nh_.getParam(this->ns_ + "/depth_intrinsics", depthIntrinsics)){
			cout << this->hint_ << ": Please check camera intrinsics!" << endl;
			exit(0);
		}
		else{
			this->fx_ = depthIntrinsics[0];
			this->fy_ = depthIntrinsics[1];
			this->cx_ = depthIntrinsics[2];
			this->cy_ = depthIntrinsics[3];
			cout << this->hint_ << ": fx, fy, cx, cy: " << "["  << this->fx_ << ", " << this->fy_  << ", " << this->cx_ << ", "<< this->cy_ << "]" << endl;
		}

		// depth scale factor
		if (not this->nh_.getParam(this->ns_ + "/depth_scale_factor", this->depthScale_)){
			this->depthScale_ = 1000.0;
			cout << this->hint_ << ": No depth scale factor. Use default: 1000." << endl;
		}
		else{
			cout << this->hint_ << ": Depth scale factor: " << this->depthScale_ << endl;
		}

		// depth min value
		if (not this->nh_.getParam(this->ns_ + "/depth_min_value", this->depthMinValue_)){
			this->depthMinValue_ = 0.2;
			cout << this->hint_ << ": No depth min value. Use default: 0.2 m." << endl;
		}
		else{
			cout << this->hint_ << ": Depth min value: " << this->depthMinValue_ << endl;
		}

		// depth max value
		if (not this->nh_.getParam(this->ns_ + "/depth_max_value", this->depthMaxValue_)){
			this->depthMaxValue_ = 5.0;
			cout << this->hint_ << ": No depth max value. Use default: 5.0 m." << endl;
		}
		else{
			cout << this->hint_ << ": Depth depth max value: " << this->depthMaxValue_ << endl;
		}

		// depth filter margin
		if (not this->nh_.getParam(this->ns_ + "/depth_filter_margin", this->depthFilterMargin_)){
			this->depthFilterMargin_ = 0;
			cout << this->hint_ << ": No depth filter margin. Use default: 0." << endl;
		}
		else{
			cout << this->hint_ << ": Depth filter margin: " << this->depthFilterMargin_ << endl;
		}

		// depth skip pixel
		if (not this->nh_.getParam(this->ns_ + "/depth_skip_pixel", this->skipPixel_)){
			this->skipPixel_ = 1;
			cout << this->hint_ << ": No depth skip pixel. Use default: 1." << endl;
		}
		else{
			cout << this->hint_ << ": Depth skip pixel: " << this->skipPixel_ << endl;
		}

		// ------------------------------------------------------------------------------------
		// depth image columns
		if (not this->nh_.getParam(this->ns_ + "/image_cols", this->imgCols_)){
			this->imgCols_ = 640;
			cout << this->hint_ << ": No depth image columns. Use default: 640." << endl;
		}
		else{
			cout << this->hint_ << ": Depth image columns: " << this->imgCols_ << endl;
		}

		// depth skip pixel
		if (not this->nh_.getParam(this->ns_ + "/image_rows", this->imgRows_)){
			this->imgRows_ = 480;
			cout << this->hint_ << ": No depth image rows. Use default: 480." << endl;
		}
		else{
			cout << this->hint_ << ": Depth image rows: " << this->imgRows_ << endl;
		}
		this->projPoints_.resize(this->imgCols_ * this->imgRows_ / (this->skipPixel_ * this->skipPixel_));
		// ------------------------------------------------------------------------------------


		// transform matrix: body to camera
		std::vector<double> body2CamVec (16);
		if (not this->nh_.getParam(this->ns_ + "/body_to_camera", body2CamVec)){
			ROS_ERROR("[OccMap]: Please check body to camera matrix!");
		}
		else{
			for (int i=0; i<4; ++i){
				for (int j=0; j<4; ++j){
					this->body2Cam_(i, j) = body2CamVec[i * 4 + j];
				}
			}
			// cout << this->hint_ << ": from body to camera: " << endl;
			// cout << this->body2Cam_ << endl;
		}

		// Raycast max length
		if (not this->nh_.getParam(this->ns_ + "/raycast_max_length", this->raycastMaxLength_)){
			this->raycastMaxLength_ = 5.0;
			cout << this->hint_ << ": No raycast max length. Use default: 5.0." << endl;
		}
		else{
			cout << this->hint_ << ": Raycast max length: " << this->raycastMaxLength_ << endl;
		}

		// p hit
		double pHit;
		if (not this->nh_.getParam(this->ns_ + "/p_hit", pHit)){
			pHit = 0.70;
			cout << this->hint_ << ": No p hit. Use default: 0.70." << endl;
		}
		else{
			cout << this->hint_ << ": P hit: " << pHit << endl;
		}
		this->pHitLog_ = this->logit(pHit);

		// p miss
		double pMiss;
		if (not this->nh_.getParam(this->ns_ + "/p_miss", pMiss)){
			pMiss = 0.35;
			cout << this->hint_ << ": No p miss. Use default: 0.35." << endl;
		}
		else{
			cout << this->hint_ << ": P miss: " << pMiss << endl;
		}
		this->pMissLog_ = this->logit(pMiss);

		// p min
		double pMin;
		if (not this->nh_.getParam(this->ns_ + "/p_min", pMin)){
			pHit = 0.12;
			cout << this->hint_ << ": No p min. Use default: 0.12." << endl;
		}
		else{
			cout << this->hint_ << ": P min: " << pMin << endl;
		}
		this->pMinLog_ = this->logit(pMin);

		// p max
		double pMax;
		if (not this->nh_.getParam(this->ns_ + "/p_max", pMax)){
			pMax = 0.97;
			cout << this->hint_ << ": No p max. Use default: 0.97." << endl;
		}
		else{
			cout << this->hint_ << ": P max: " << pMax << endl;
		}
		this->pMaxLog_ = this->logit(pMax);

		// p occ
		double pOcc;
		if (not this->nh_.getParam(this->ns_ + "/p_occ", pOcc)){
			pOcc = 0.80;
			cout << this->hint_ << ": No p occ. Use default: 0.80." << endl;
		}
		else{
			cout << this->hint_ << ": P occ: " << pOcc << endl;
		}
		this->pOccLog_ = this->logit(pOcc);


		// map resolution
		if (not this->nh_.getParam(this->ns_ + "/map_resolution", this->mapRes_)){
			this->mapRes_ = 0.1;
			cout << this->hint_ << ": No map resolution. Use default: 0.1." << endl;
		}
		else{
			cout << this->hint_ << ": Map resolution: " << this->mapRes_ << endl;
		}

		// ground height
		if (not this->nh_.getParam(this->ns_ + "/ground_height", this->groundHeight_)){
			this->groundHeight_ = 0.0;
			cout << this->hint_ << ": No ground height. Use default: 0.0." << endl;
		}
		else{
			cout << this->hint_ << ": Ground height: " << this->groundHeight_ << endl;
		}


		// map size
		std::vector<double> mapSizeVec (3);
		if (not this->nh_.getParam(this->ns_ + "/map_size", mapSizeVec)){
			mapSizeVec[0] = 20; mapSizeVec[1] = 20; mapSizeVec[2] = 3;
			cout << this->hint_ << ": No map size. Use default: [20, 20, 3]." << endl;
		}
		else{
			this->mapSize_(0) = mapSizeVec[0];
			this->mapSize_(1) = mapSizeVec[1];
			this->mapSize_(2) = mapSizeVec[2];

			// init min max
			this->mapSizeMin_(0) = -mapSizeVec[0]/2; this->mapSizeMax_(0) = mapSizeVec[0]/2;
			this->mapSizeMin_(1) = -mapSizeVec[1]/2; this->mapSizeMax_(1) = mapSizeVec[1]/2;
			this->mapSizeMin_(2) = this->groundHeight_; this->mapSizeMax_(2) = this->groundHeight_ + mapSizeVec[2];
			
			// min max for voxel
			this->mapVoxelMin_(0) = 0; this->mapVoxelMax_(0) = ceil(mapSizeVec[0]/this->mapRes_);
			this->mapVoxelMin_(1) = 0; this->mapVoxelMax_(1) = ceil(mapSizeVec[1]/this->mapRes_);
			this->mapVoxelMin_(2) = 0; this->mapVoxelMax_(2) = ceil(mapSizeVec[2]/this->mapRes_);

			// reserve vector for variables
			int reservedSize = this->mapVoxelMax_(0) * this->mapVoxelMax_(1) * this->mapVoxelMax_(2);
			this->countHitMiss_.resize(reservedSize, 0);
			this->countHit_.resize(reservedSize, 0);
			this->occupancy_.resize(reservedSize, this->pMinLog_-this->UNKNOWN_FLAG_);
			this->occupancyInflated_.resize(reservedSize, false);
			this->flagTraverse_.resize(reservedSize, -1);
			this->flagRayend_.resize(reservedSize, -1);

			cout << this->hint_ << ": Map size: " << "[" << mapSizeVec[0] << ", " << mapSizeVec[1] << ", " << mapSizeVec[2] << "]" << endl;
		}

		// local update range
		std::vector<double> localUpdateRangeVec;
		if (not this->nh_.getParam(this->ns_ + "/local_update_range", localUpdateRangeVec)){
			localUpdateRangeVec = std::vector<double>{5.0, 5.0, 3.0};
			cout << this->hint_ << ": No local update range. Use default: [5.0, 5.0, 3.0] m." << endl;
		}
		else{
			cout << this->hint_ << ": Local update range: " << "[" << localUpdateRangeVec[0] << ", " << localUpdateRangeVec[1] << ", " << localUpdateRangeVec[2] << "]" << endl;
		}
		this->localUpdateRange_(0) = localUpdateRangeVec[0]; this->localUpdateRange_(1) = localUpdateRangeVec[1]; this->localUpdateRange_(2) = localUpdateRangeVec[2];


		// local bound inflate factor
		if (not this->nh_.getParam(this->ns_ + "/local_bound_inflation", this->localBoundInflate_)){
			this->localBoundInflate_ = 0.0;
			cout << this->hint_ << ": No local bound inflate. Use default: 0.0 m." << endl;
		}
		else{
			cout << this->hint_ << ": Local bound inflate: " << this->localBoundInflate_ << endl;
		}

		// whether to clean local map
		if (not this->nh_.getParam(this->ns_ + "/clean_local_map", this->cleanLocalMap_)){
			this->cleanLocalMap_ = true;
			cout << this->hint_ << ": No clean local map option. Use default: true." << endl;
		}
		else{
			cout << this->hint_ << ": Clean local map option is set to: " << this->cleanLocalMap_ << endl; 
		}

		// local map size (visualization)
		std::vector<double> localMapSizeVec;
		if (not this->nh_.getParam(this->ns_ + "/local_map_size", localMapSizeVec)){
			localMapSizeVec = std::vector<double>{10.0, 10.0, 2.0};
			cout << this->hint_ << ": No local map size. Use default: [10.0, 10.0, 3.0] m." << endl;
		}
		else{
			cout << this->hint_ << ": Local map size: " << "[" << localMapSizeVec[0] << ", " << localMapSizeVec[1] << ", " << localMapSizeVec[2] << "]" << endl;
		}
		this->localMapSize_(0) = localMapSizeVec[0]/2; this->localMapSize_(1) = localMapSizeVec[1]/2; this->localMapSize_(2) = localMapSizeVec[2]/2;
		this->localMapVoxel_(0) = int(ceil(localMapSizeVec[0]/(2*this->mapRes_))); this->localMapVoxel_(1) = int(ceil(localMapSizeVec[1]/(2*this->mapRes_))); this->localMapVoxel_(2) = int(ceil(localMapSizeVec[2]/(2*this->mapRes_)));

		// max vis height
		if (not this->nh_.getParam(this->ns_ + "/max_height_visualization", this->maxVisHeight_)){
			this->maxVisHeight_ = 3.0;
			cout << this->hint_ << ": No max visualization height. Use default: 3.0 m." << endl;
		}
		else{
			cout << this->hint_ << ": Max visualization height: " << this->maxVisHeight_ << endl;
		}

		// max vis height
		if (not this->nh_.getParam(this->ns_ + "/visualize_global_map", this->visGlobalMap_)){
			this->visGlobalMap_ = false;
			cout << this->hint_ << ": No visualize map option. Use default: visualize local map." << endl;
		}
		else{
			cout << this->hint_ << ": Visualize map option. local (0)/global (1): " << this->visGlobalMap_ << endl;
		}


	}

	void occMap::registerCallback(){
		// depth pose callback
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
			ROS_ERROR("[OccMap]: Invalid localization mode!");
			exit(0);
		}

		// occupancy update callback
		this->occTimer_ = this->nh_.createTimer(ros::Duration(0.05), &occMap::updateOccupancyCB, this);

		// map inflation callback
		this->inflateTimer_ = this->nh_.createTimer(ros::Duration(0.05), &occMap::inflateMapCB, this);

		// visualization callback
		this->visTimer_ = this->nh_.createTimer(ros::Duration(0.05), &occMap::visCB, this);
	}

	void occMap::registerPub(){
		this->depthCloudPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>(this->ns_ + "/depth_cloud", 10);
		this->mapVisPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>(this->ns_ + "/voxel_map", 10);
		this->inflatedMapVisPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>(this->ns_ + "/inflated_voxel_map", 10);
	}


	void occMap::depthPoseCB(const sensor_msgs::ImageConstPtr& img, const geometry_msgs::PoseStampedConstPtr& pose){
		// store current depth image
		cv_bridge::CvImagePtr imgPtr = cv_bridge::toCvCopy(img, img->encoding);
		if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1){
			(imgPtr->image).convertTo(imgPtr->image, CV_16UC1, this->depthScale_);
		}
		imgPtr->image.copyTo(this->depthImage_);

		// store current position and orientation (camera)
		Eigen::Matrix4d camPoseMatrix;
		this->getCameraPose(pose, camPoseMatrix);

		this->position_(0) = camPoseMatrix(0, 3);
		this->position_(1) = camPoseMatrix(1, 3);
		this->position_(2) = camPoseMatrix(2, 3);
		this->orientation_ = camPoseMatrix.block<3, 3>(0, 0);

		if (this->isInMap(this->position_)){
			this->occNeedUpdate_ = true;
		}
		else{
			this->occNeedUpdate_ = false;
		}
	}

	void occMap::depthOdomCB(const sensor_msgs::ImageConstPtr& img, const nav_msgs::OdometryConstPtr& odom){
		// store current depth image
		cv_bridge::CvImagePtr imgPtr = cv_bridge::toCvCopy(img, img->encoding);
		if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1){
			(imgPtr->image).convertTo(imgPtr->image, CV_16UC1, this->depthScale_);
		}
		imgPtr->image.copyTo(this->depthImage_);

		// store current position and orientation (camera)
		Eigen::Matrix4d camPoseMatrix;
		this->getCameraPose(odom, camPoseMatrix);

		this->position_(0) = camPoseMatrix(0, 3);
		this->position_(1) = camPoseMatrix(1, 3);
		this->position_(2) = camPoseMatrix(2, 3);
		this->orientation_ = camPoseMatrix.block<3, 3>(0, 0);

		if (this->isInMap(this->position_)){
			this->occNeedUpdate_ = true;
		}
		else{
			this->occNeedUpdate_ = false;
		}
	}

	void occMap::updateOccupancyCB(const ros::TimerEvent& ){
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
		// this->inflateLocalMap();
		endTime = ros::Time::now();
		
		cout << this->hint_ << ": Occupancy update time: " << (endTime - startTime).toSec() << " s." << endl;
		this->occNeedUpdate_ = false;
		this->mapNeedInflate_ = true;
	}

	void occMap::inflateMapCB(const ros::TimerEvent& ){
		// inflate local map:
		if (this->mapNeedInflate_){
			this->inflateLocalMap();
			this->mapNeedInflate_ = false;
			this->esdfNeedUpdate_ = true;
		}
	}


	void occMap::projectDepthImage(){
		this->projPointsNum_ = 0;

		int cols = this->depthImage_.cols;
		int rows = this->depthImage_.rows;
		uint16_t* rowPtr;

		Eigen::Vector3d currPointCam, currPointMap;
		double depth;
		const double inv_factor = 1.0 / this->depthScale_;
		const double inv_fx = 1.0 / this->fx_;
		const double inv_fy = 1.0 / this->fy_;


		// iterate through each pixel in the depth image
		for (int v=this->depthFilterMargin_; v<rows-this->depthFilterMargin_; v=v+this->skipPixel_){ // row
			rowPtr = this->depthImage_.ptr<uint16_t>(v) + this->depthFilterMargin_;
			for (int u=this->depthFilterMargin_; u<cols-this->depthFilterMargin_; u=u+this->skipPixel_){ // column
				depth = (*rowPtr) * inv_factor;
				
				if (*rowPtr == 0) {
					depth = this->raycastMaxLength_ + 0.1;
				} else if (depth < this->depthMinValue_) {
					continue;
				} else if (depth > this->depthMaxValue_) {
					depth = this->raycastMaxLength_ + 0.1;
				}
				rowPtr =  rowPtr + this->skipPixel_;

				// get 3D point in camera frame
				currPointCam(0) = (u - this->cx_) * depth * inv_fx;
				currPointCam(1) = (v - this->cy_) * depth * inv_fy;
				currPointCam(2) = depth;
				currPointMap = this->orientation_ * currPointCam + this->position_; // transform to map coordinate

				// store current point
				this->projPoints_[this->projPointsNum_] = currPointMap;
				this->projPointsNum_ = this->projPointsNum_ + 1;
			}
		} 
	}


	void occMap::raycastUpdate(){
		if (this->projPointsNum_ == 0){
			return;
		}
		this->raycastNum_ += 1;

		// record local bound of update
		double xmin, xmax, ymin, ymax, zmin, zmax;
		xmin = xmax = this->position_(0);
		ymin = ymax = this->position_(1);
		zmin = zmax = this->position_(2);

		// iterate through each projected points, perform raycasting and update occupancy
		Eigen::Vector3d currPoint;
		bool pointAdjusted;
		int rayendVoxelID, raycastVoxelID;
		double length;
		for (int i=0; i<this->projPointsNum_; ++i){
			currPoint = this->projPoints_[i];

			pointAdjusted = false;
			// check whether the point is in reserved map range
			if (not this->isInMap(currPoint)){
				currPoint = this->adjustPointInMap(currPoint);
				pointAdjusted = true;
			}

			// check whether the point exceeds the maximum raycasting length
			length = (currPoint - this->position_).norm();
			if (length > this->raycastMaxLength_){
				currPoint = this->adjustPointRayLength(currPoint);
				pointAdjusted = true;
			}


			// update local bound
			if (currPoint(0) < xmin){xmin = currPoint(0);}
			if (currPoint(1) < ymin){ymin = currPoint(1);}
			if (currPoint(2) < zmin){zmin = currPoint(2);}
			if (currPoint(0) > xmax){xmax = currPoint(0);}
			if (currPoint(1) > ymax){ymax = currPoint(1);}
			if (currPoint(2) > zmax){zmax = currPoint(2);}

			// update occupancy itself update information
			rayendVoxelID = this->updateOccupancyInfo(currPoint, not pointAdjusted); // point adjusted is free, not is occupied

			// check whether the voxel has already been updated, so no raycasting needed
			// rayendVoxelID = this->posToAddress(currPoint);
			if (this->flagRayend_[rayendVoxelID] == this->raycastNum_){
				continue; // skip
			}
			else{
				this->flagRayend_[rayendVoxelID] = this->raycastNum_;
			}



			// raycasting for update occupancy
			this->raycaster_.setInput(currPoint/this->mapRes_, this->position_/this->mapRes_);
			Eigen::Vector3d rayPoint, actualPoint;
			while (this->raycaster_.step(rayPoint)){
				actualPoint = rayPoint;
				actualPoint(0) += 0.5;
				actualPoint(1) += 0.5;
				actualPoint(2) += 0.5;
				actualPoint *= this->mapRes_;
				raycastVoxelID = this->updateOccupancyInfo(actualPoint, false);

				// raycastVoxelID = this->posToAddress(actualPoint);
				if (this->flagTraverse_[raycastVoxelID] == this->raycastNum_){
					break;
				}
				else{
					this->flagTraverse_[raycastVoxelID] = this->raycastNum_;
				}

			}
		}

		// store local bound and inflate local bound (inflate is for ESDF update)
		this->posToIndex(Eigen::Vector3d (xmin, ymin, zmin), this->localBoundMin_);
		this->posToIndex(Eigen::Vector3d (xmax, ymax, zmax), this->localBoundMax_);
		this->localBoundMin_ -= int(ceil(this->localBoundInflate_/this->mapRes_)) * Eigen::Vector3i(1, 1, 0); // inflate in x y direction
		this->localBoundMax_ += int(ceil(this->localBoundInflate_/this->mapRes_)) * Eigen::Vector3i(1, 1, 0); 
		this->boundIndex(this->localBoundMin_); // since inflated, need to bound if not in reserved range
		this->boundIndex(this->localBoundMax_);


		// update occupancy in the cache
		double logUpdateValue;
		int cacheAddress, hit, miss;
		while (not this->updateVoxelCache_.empty()){
			Eigen::Vector3i cacheIdx = this->updateVoxelCache_.front();
			this->updateVoxelCache_.pop();
			cacheAddress = this->indexToAddress(cacheIdx);

			hit = this->countHit_[cacheAddress];
			miss = this->countHitMiss_[cacheAddress] - hit;

			if (hit >= miss){
				logUpdateValue = this->pHitLog_;
			}
			else{
				logUpdateValue = this->pMissLog_;
			}
			this->countHit_[cacheAddress] = 0; // clear hit
			this->countHitMiss_[cacheAddress] = 0; // clear hit and miss

			// check whether point is in the local update range
			if (not this->isInLocalUpdateRange(cacheIdx)){
				continue; // do not update if not in the range
			}

			// update occupancy info
			if ((logUpdateValue >= 0) and (this->occupancy_[cacheAddress] >= this->pMaxLog_)){
				continue; // not increase p if max clamped
			}
			else if ((logUpdateValue <= 0) and (this->occupancy_[cacheAddress] == this->pMinLog_)){
				continue; // not decrease p if min clamped
			}
			else if ((logUpdateValue <= 0) and (this->occupancy_[cacheAddress] < this->pMinLog_)){
				this->occupancy_[cacheAddress] = this->pMinLog_; // if unknown set it free (prior), 
				continue;
			}

			this->occupancy_[cacheAddress] = std::min(std::max(this->occupancy_[cacheAddress]+logUpdateValue, this->pMinLog_), this->pMaxLog_);
		}

	}

	void occMap::cleanLocalMap(){
		Eigen::Vector3i posIndex;
		this->posToIndex(this->position_, posIndex);
		Eigen::Vector3i innerMinBBX = posIndex - this->localMapVoxel_;
		Eigen::Vector3i innerMaxBBX = posIndex + this->localMapVoxel_;
		Eigen::Vector3i outerMinBBX = innerMinBBX - Eigen::Vector3i(5, 5, 5);
		Eigen::Vector3i outerMaxBBX = innerMaxBBX + Eigen::Vector3i(5, 5, 5);
		this->boundIndex(innerMinBBX);
		this->boundIndex(innerMaxBBX);
		this->boundIndex(outerMinBBX);
		this->boundIndex(outerMaxBBX);

		// clear x axis
		for (int y=outerMinBBX(1); y<=outerMaxBBX(1); ++y){
			for (int z=outerMinBBX(2); z<=outerMaxBBX(2); ++z){
				for (int x=outerMinBBX(0); x<=innerMinBBX(0); ++x){
					this->occupancy_[this->indexToAddress(Eigen::Vector3i (x, y, z))] = this->pMinLog_ - this->UNKNOWN_FLAG_;
				}

				for (int x=innerMaxBBX(0); x<=outerMaxBBX(0); ++x){
					this->occupancy_[this->indexToAddress(Eigen::Vector3i (x, y, z))] = this->pMinLog_ - this->UNKNOWN_FLAG_;					
				}
			}
		}

		// clear y axis
		for (int x=outerMinBBX(0); x<=outerMaxBBX(0); ++x){
			for (int z=outerMinBBX(2); z<=outerMaxBBX(2); ++z){
				for (int y=outerMinBBX(1); y<=innerMinBBX(1); ++y){
					this->occupancy_[this->indexToAddress(Eigen::Vector3i (x, y, z))] = this->pMinLog_ - this->UNKNOWN_FLAG_;
				}

				for (int y=innerMaxBBX(1); y<=outerMaxBBX(1); ++y){
					this->occupancy_[this->indexToAddress(Eigen::Vector3i (x, y, z))] = this->pMinLog_ - this->UNKNOWN_FLAG_;
				}
			}
		}

		// clear z axis
		for (int x=outerMinBBX(0); x<=outerMaxBBX(0); ++x){
			for (int y=outerMinBBX(1); y<=outerMaxBBX(1); ++y){
				for (int z=outerMinBBX(2); z<=innerMinBBX(2); ++z){
					this->occupancy_[this->indexToAddress(Eigen::Vector3i (x, y, z))] = this->pMinLog_ - this->UNKNOWN_FLAG_;
				}

				for (int z=innerMaxBBX(2); z<=outerMaxBBX(2); ++z){
					this->occupancy_[this->indexToAddress(Eigen::Vector3i (x, y, z))] = this->pMinLog_ - this->UNKNOWN_FLAG_;
				}
			}
		}
	}

	void occMap::inflateLocalMap(){
		Eigen::Vector3i clearIndex;
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

	inline bool occMap::isOccupied(const Eigen::Vector3d& pos){
		Eigen::Vector3i idx;
		this->posToIndex(pos, idx);
		return this->isOccupied(idx);
	}

	inline bool occMap::isOccupied(const Eigen::Vector3i& idx){
		int address = this->indexToAddress(idx);
		return this->occupancy_[address] >= this->pOccLog_;
	}

	inline bool occMap::isInflatedOccupied(const Eigen::Vector3d& pos){
		Eigen::Vector3i idx;
		this->posToIndex(pos, idx);
		return this->isInflatedOccupied(idx);
	}

	inline bool occMap::isInflatedOccupied(const Eigen::Vector3i& idx){
		int address = this->indexToAddress(idx);
		return this->occupancyInflated_[address] == true;
	}

	inline bool occMap::isFree(const Eigen::Vector3d& pos){
		Eigen::Vector3i idx;
		this->posToIndex(pos, idx);
		return this->isFree(idx);
	}

	inline bool occMap::isFree(const Eigen::Vector3i& idx){
		int address = this->indexToAddress(idx);
		return (this->occupancy_[address] < this->pOccLog_) and (this->occupancy_[address] >= this->pMinLog_);
	}

	inline bool occMap::isUnknown(const Eigen::Vector3d& pos){
		Eigen::Vector3i idx;
		this->posToIndex(pos, idx);
		return this->isUnknown(idx);
	}

	inline bool occMap::isUnknown(const Eigen::Vector3i& idx){
		int address = this->indexToAddress(idx);
		return this->occupancy_[address] < this->pMinLog_;		
	}

	void occMap::visCB(const ros::TimerEvent& ){
		this->publishProjPoints();
		this->publishMap();
		this->publishInflatedMap();
	}

	void occMap::publishProjPoints(){
		pcl::PointXYZ pt;
		pcl::PointCloud<pcl::PointXYZ> cloud;

		for (int i=0; i<this->projPointsNum_; ++i){
			pt.x = this->projPoints_[i](0);
			pt.y = this->projPoints_[i](1);
			pt.z = this->projPoints_[i](2);
			cloud.push_back(pt);
		}

		cloud.width = cloud.points.size();
		cloud.height = 1;
		cloud.is_dense = true;
		cloud.header.frame_id = "map";

		sensor_msgs::PointCloud2 cloudMsg;
		pcl::toROSMsg(cloud, cloudMsg);
		this->depthCloudPub_.publish(cloudMsg);
	}


	void occMap::publishMap(){
		pcl::PointXYZ pt;
		pcl::PointCloud<pcl::PointXYZ> cloud;

		Eigen::Vector3d minRange, maxRange;
		if (this->visGlobalMap_){
			minRange = this->mapSizeMin_;
			maxRange = this->mapSizeMax_;
		}
		else{
			minRange = this->position_ - localMapSize_;
			maxRange = this->position_ + localMapSize_;
			minRange(2) = this->groundHeight_;
		}
		Eigen::Vector3i minRangeIdx, maxRangeIdx;
		this->posToIndex(minRange, minRangeIdx);
		this->posToIndex(maxRange, maxRangeIdx);
		this->boundIndex(minRangeIdx);
		this->boundIndex(maxRangeIdx);

		for (int x=minRangeIdx(0); x<=maxRangeIdx(0); ++x){
			for (int y=minRangeIdx(1); y<=maxRangeIdx(1); ++y){
				for (int z=minRangeIdx(2); z<=maxRangeIdx(2); ++z){
					Eigen::Vector3i pointIdx (x, y, z);
					// if (this->occupancy_[this->indexToAddress(pointIdx)] > this->pMinLog_){
					if (this->isOccupied(pointIdx)){
						Eigen::Vector3d point;
						this->indexToPos(pointIdx, point);
						if (point(2) <= this->maxVisHeight_){
							pt.x = point(0);
							pt.y = point(1);
							pt.z = point(2);
							cloud.push_back(pt);
						}
						if (point(2) > 2.9){
							cout << "here" << endl;
							cout << pointIdx << endl;
							exit(0);
						}
					}
				}
			}
		}

		cloud.width = cloud.points.size();
		cloud.height = 1;
		cloud.is_dense = true;
		cloud.header.frame_id = "map";

		sensor_msgs::PointCloud2 cloudMsg;
		pcl::toROSMsg(cloud, cloudMsg);
		this->mapVisPub_.publish(cloudMsg);
	}

	void occMap::publishInflatedMap(){
		pcl::PointXYZ pt;
		pcl::PointCloud<pcl::PointXYZ> cloud;

		Eigen::Vector3d minRange, maxRange;
		if (this->visGlobalMap_){
			minRange = this->mapSizeMin_;
			maxRange = this->mapSizeMax_;
		}
		else{
			minRange = this->position_ - localMapSize_;
			maxRange = this->position_ + localMapSize_;
			minRange(2) = this->groundHeight_;
		}
		Eigen::Vector3i minRangeIdx, maxRangeIdx;
		this->posToIndex(minRange, minRangeIdx);
		this->posToIndex(maxRange, maxRangeIdx);
		this->boundIndex(minRangeIdx);
		this->boundIndex(maxRangeIdx);

		for (int x=minRangeIdx(0); x<=maxRangeIdx(0); ++x){
			for (int y=minRangeIdx(1); y<=maxRangeIdx(1); ++y){
				for (int z=minRangeIdx(2); z<=maxRangeIdx(2); ++z){
					Eigen::Vector3i pointIdx (x, y, z);
					// if (this->occupancy_[this->indexToAddress(pointIdx)] > this->pMinLog_){
					if (this->isInflatedOccupied(pointIdx)){
						Eigen::Vector3d point;
						this->indexToPos(pointIdx, point);
						if (point(2) <= this->maxVisHeight_){
							pt.x = point(0);
							pt.y = point(1);
							pt.z = point(2);
							cloud.push_back(pt);
						}
					}
				}
			}
		}

		cloud.width = cloud.points.size();
		cloud.height = 1;
		cloud.is_dense = true;
		cloud.header.frame_id = "map";

		sensor_msgs::PointCloud2 cloudMsg;
		pcl::toROSMsg(cloud, cloudMsg);
		this->inflatedMapVisPub_.publish(cloudMsg);	
	}


	inline bool occMap::isInMap(const Eigen::Vector3d& pos){
		if ((pos(0) >= this->mapSizeMin_(0)) and (pos(0) <= this->mapSizeMax_(0)) and 
			(pos(1) >= this->mapSizeMin_(1)) and (pos(1) <= this->mapSizeMax_(1)) and 
			(pos(2) >= this->mapSizeMin_(2)) and (pos(2) <= this->mapSizeMax_(2))){
			return true;
		}
		else{
			return false;
		}
	}

	inline bool occMap::isInMap(const Eigen::Vector3i& idx){
		if ((idx(0) >= this->mapVoxelMin_(0)) and (idx(0) < this->mapVoxelMax_(0)) and
		    (idx(1) >= this->mapVoxelMin_(1)) and (idx(1) < this->mapVoxelMax_(1)) and 
		    (idx(2) >= this->mapVoxelMin_(2)) and (idx(2) < this->mapVoxelMax_(2))){
			return true;
		}
		else{
			return false;
		}
	}

	inline void occMap::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& idx){
		idx(0) = floor( (pos(0) - this->mapSizeMin_(0) ) / this->mapRes_ );
		idx(1) = floor( (pos(1) - this->mapSizeMin_(1) ) / this->mapRes_ );
		idx(2) = floor( (pos(2) - this->mapSizeMin_(2) ) / this->mapRes_ );
	}

	inline void occMap::indexToPos(const Eigen::Vector3i& idx, Eigen::Vector3d& pos){
		pos(0) = (idx(0) + 0.5) * this->mapRes_ + this->mapSizeMin_(0); 
		pos(1) = (idx(1) + 0.5) * this->mapRes_ + this->mapSizeMin_(1);
		pos(2) = (idx(2) + 0.5) * this->mapRes_ + this->mapSizeMin_(2);
	}

	inline int occMap::posToAddress(const Eigen::Vector3d& pos){
		Eigen::Vector3i idx;
		this->posToIndex(pos, idx);
		return this->indexToAddress(idx);
	}

	inline int occMap::posToAddress(double x, double y, double z){
		Eigen::Vector3d pos (x, y, z);
		return this->posToAddress(pos);
	}

	inline int occMap::indexToAddress(const Eigen::Vector3i& idx){
		return idx(0) * this->mapVoxelMax_(1) * this->mapVoxelMax_(2) + idx(1) * this->mapVoxelMax_(2) + idx(2);
	}

	inline int occMap::indexToAddress(int x, int y, int z){
		Eigen::Vector3i idx (x, y, z);
		return this->indexToAddress(idx);
	}

	inline void occMap::boundIndex(Eigen::Vector3i& idx){
		Eigen::Vector3i temp;
		temp(0) = std::max(std::min(idx(0), this->mapVoxelMax_(0)-1), this->mapVoxelMin_(0));
		temp(1) = std::max(std::min(idx(1), this->mapVoxelMax_(1)-1), this->mapVoxelMin_(1));
		temp(2) = std::max(std::min(idx(2), this->mapVoxelMax_(2)-1), this->mapVoxelMin_(2));
		idx = temp;
	}

	inline bool occMap::isInLocalUpdateRange(const Eigen::Vector3d& pos){
		Eigen::Vector3i idx;
		this->posToIndex(pos, idx);
		return this->isInLocalUpdateRange(idx);
	}

	inline bool occMap::isInLocalUpdateRange(const Eigen::Vector3i& idx){
		Eigen::Vector3d rangeMin = this->position_ - this->localUpdateRange_;
		Eigen::Vector3d rangeMax = this->position_ + this->localUpdateRange_;
		
		Eigen::Vector3i rangeMinIdx, rangeMaxIdx;
		this->posToIndex(rangeMin, rangeMinIdx);
		this->posToIndex(rangeMax, rangeMaxIdx);

		this->boundIndex(rangeMinIdx);
		this->boundIndex(rangeMaxIdx);

		bool inRange = (idx(0) >= rangeMinIdx(0)) and (idx(0) <= rangeMaxIdx(0)) and
					   (idx(1) >= rangeMinIdx(1)) and (idx(1) <= rangeMaxIdx(1)) and
					   (idx(2) >= rangeMinIdx(2)) and (idx(2) <= rangeMaxIdx(2));
		return inRange;
	}

	inline Eigen::Vector3d occMap::adjustPointInMap(const Eigen::Vector3d& point){
		Eigen::Vector3d pos = this->position_;
		Eigen::Vector3d diff = point - pos;
		Eigen::Vector3d offsetMin = this->mapSizeMin_ - pos;
		Eigen::Vector3d offsetMax = this->mapSizeMax_ - pos;

		double minRatio = 10000000;
		for (int i=0; i<3; ++i){ // each axis
			if (diff[i] != 0){
				double ratio1 = offsetMin[i]/diff[i];
				double ratio2 = offsetMax[i]/diff[i];
				if ((ratio1 > 0) and (ratio1 < minRatio)){
					minRatio = ratio1;
				}

				if ((ratio2 > 0) and (ratio2 < minRatio)){
					minRatio = ratio2;
				}
			}
		}

		return pos + (minRatio - 1e-3) * diff;
	}


	inline Eigen::Vector3d occMap::adjustPointRayLength(const Eigen::Vector3d& point){
		double length = (point - this->position_).norm();
		return (point - this->position_) * (this->raycastMaxLength_/length) + this->position_;
	}

	inline int occMap::updateOccupancyInfo(const Eigen::Vector3d& point, bool isOccupied){
		Eigen::Vector3i idx;
		this->posToIndex(point, idx);
		int voxelID = this->indexToAddress(idx);
		this->countHitMiss_[voxelID] += 1;
		if (this->countHitMiss_[voxelID] == 1){
			this->updateVoxelCache_.push(idx);
		}
		if (isOccupied){ // if not adjusted set it to occupied, otherwise it is free
			this->countHit_[voxelID] += 1;
		}
		return voxelID;
	}

	double occMap::logit(double x){
		return log(x/(1-x));
	}

	inline void occMap::getCameraPose(const geometry_msgs::PoseStampedConstPtr& pose, Eigen::Matrix4d& camPoseMatrix){
		Eigen::Quaterniond quat;
		quat = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x, pose->pose.orientation.y, pose->pose.orientation.z);
		Eigen::Matrix3d rot = quat.toRotationMatrix();

		// convert body pose to camera pose
		Eigen::Matrix4d map2body; map2body.setZero();
		map2body.block<3, 3>(0, 0) = rot;
		map2body(0, 3) = pose->pose.position.x; 
		map2body(1, 3) = pose->pose.position.y;
		map2body(2, 3) = pose->pose.position.z;
		map2body(3, 3) = 1.0;

		camPoseMatrix = map2body * this->body2Cam_;
	}

	inline void occMap::getCameraPose(const nav_msgs::OdometryConstPtr& odom, Eigen::Matrix4d& camPoseMatrix){
		Eigen::Quaterniond quat;
		quat = Eigen::Quaterniond(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
		Eigen::Matrix3d rot = quat.toRotationMatrix();

		// convert body pose to camera pose
		Eigen::Matrix4d map2body; map2body.setZero();
		map2body.block<3, 3>(0, 0) = rot;
		map2body(0, 3) = odom->pose.pose.position.x; 
		map2body(1, 3) = odom->pose.pose.position.y;
		map2body(2, 3) = odom->pose.pose.position.z;
		map2body(3, 3) = 1.0;

		camPoseMatrix = map2body * this->body2Cam_;
	}
}