/*
	FILE: occupancyMap.cpp
	--------------------------------------
	function definition of occupancy map
*/
#include <map_manager/occupancyMap.h>

namespace mapManager{
	occMap::occMap(){}

	void occMap::initMap(const ros::NodeHandle& nh){
		this->nh_ = nh;
		this->initParam();
		this->registerPub();
		this->registerCallback();
	}

	void occMap::initParam(){
		// depth topic name
		if (not this->nh_.getParam("occupancy_map/depth_image_topic", this->depthTopicName_)){
			this->depthTopicName_ = "/camera/depth/image_raw";
			cout << "[OccMap]: No depth image topic name. Use default: /camera/depth/image_raw" << endl;
		}
		else{
			cout << "[OccMap]: Depth topic: " << this->depthTopicName_ << endl;
		}

		// pose topic name
		if (not this->nh_.getParam("occupancy_map/pose_topic", this->poseTopicName_)){
			this->poseTopicName_ = "/CERLAB/quadcopter/pose";
			cout << "[OccMap]: No pose topic name. Use default: /CERLAB/quadcopter/pose" << endl;
		}
		else{
			cout << "[OccMap]: Pose topic: " << this->poseTopicName_ << endl;
		}

		std::vector<double> depthIntrinsics (4);
		if (not this->nh_.getParam("occupancy_map/depth_intrinsics", depthIntrinsics)){
			ROS_ERROR("[OccMap]: Please check camera intrinsics!");
		}
		else{
			this->fx_ = depthIntrinsics[0];
			this->fy_ = depthIntrinsics[1];
			this->cx_ = depthIntrinsics[2];
			this->cy_ = depthIntrinsics[3];
			cout << "[OccMap]: fx, fy, cx, cy: " << "["  << this->fx_ << ", " << this->fy_  << ", " << this->cx_ << ", "<< this->cy_ << "]" << endl;
		}

		// depth scale factor
		if (not this->nh_.getParam("occupancy_map/depth_scale_factor", this->depthScale_)){
			this->depthScale_ = 1000.0;
			cout << "[OccMap]: No depth scale factor. Use default: 1000." << endl;
		}
		else{
			cout << "[OccMap]: Depth scale factor: " << this->depthScale_ << endl;
		}

		// depth min value
		if (not this->nh_.getParam("occupancy_map/depth_min_value", this->depthMinValue_)){
			this->depthMinValue_ = 0.2;
			cout << "[OccMap]: No depth min value. Use default: 0.2 m." << endl;
		}
		else{
			cout << "[OccMap]: Depth min value: " << this->depthMinValue_ << endl;
		}

		// depth max value
		if (not this->nh_.getParam("occupancy_map/depth_max_value", this->depthMaxValue_)){
			this->depthMaxValue_ = 5.0;
			cout << "[OccMap]: No depth max value. Use default: 5.0 m." << endl;
		}
		else{
			cout << "[OccMap]: Depth depth max value: " << this->depthMaxValue_ << endl;
		}

		// depth filter margin
		if (not this->nh_.getParam("occupancy_map/depth_filter_margin", this->depthFilterMargin_)){
			this->depthFilterMargin_ = 0;
			cout << "[OccMap]: No depth filter margin. Use default: 0." << endl;
		}
		else{
			cout << "[OccMap]: Depth filter margin: " << this->depthFilterMargin_ << endl;
		}

		// depth skip pixel
		if (not this->nh_.getParam("occupancy_map/depth_skip_pixel", this->skipPixel_)){
			this->skipPixel_ = 1;
			cout << "[OccMap]: No depth skip pixel. Use default: 1." << endl;
		}
		else{
			cout << "[OccMap]: Depth skip pixel: " << this->skipPixel_ << endl;
		}

		// ------------------------------------------------------------------------------------
		// depth image columns
		if (not this->nh_.getParam("occupancy_map/image_cols", this->imgCols_)){
			this->imgCols_ = 640;
			cout << "[OccMap]: No depth image columns. Use default: 640." << endl;
		}
		else{
			cout << "[OccMap]: Depth image columns: " << this->imgCols_ << endl;
		}

		// depth skip pixel
		if (not this->nh_.getParam("occupancy_map/image_rows", this->imgRows_)){
			this->imgRows_ = 480;
			cout << "[OccMap]: No depth image rows. Use default: 480." << endl;
		}
		else{
			cout << "[OccMap]: Depth image rows: " << this->imgRows_ << endl;
		}
		this->projPoints_.resize(this->imgCols_ * this->imgRows_ / (this->skipPixel_ * this->skipPixel_));
		// ------------------------------------------------------------------------------------


		// transform matrix: body to camera
		std::vector<double> body2CamVec (16);
		if (not this->nh_.getParam("occupancy_map/body_to_camera", body2CamVec)){
			ROS_ERROR("[OccMap]: Please check body to camera matrix!");
		}
		else{
			for (int i=0; i<4; ++i){
				for (int j=0; j<4; ++j){
					this->body2Cam_(i, j) = body2CamVec[i * 4 + j];
				}
			}
			// cout << "[OccMap]: from body to camera: " << endl;
			// cout << this->body2Cam_ << endl;
		}

		// Raycast max length
		if (not this->nh_.getParam("occupancy_map/raycast_max_length", this->raycastMaxLength_)){
			this->raycastMaxLength_ = 5.0;
			cout << "[OccMap]: No raycast max length. Use default: 5.0." << endl;
		}
		else{
			cout << "[OccMap]: Raycast max length: " << this->raycastMaxLength_ << endl;
		}

		// p hit
		double pHit;
		if (not this->nh_.getParam("occupancy_map/p_hit", pHit)){
			pHit = 0.70;
			cout << "[OccMap]: No p hit. Use default: 0.70." << endl;
		}
		else{
			cout << "[OccMap]: P hit: " << pHit << endl;
		}
		this->pHitLog_ = this->logit(pHit);

		// p miss
		double pMiss;
		if (not this->nh_.getParam("occupancy_map/p_miss", pMiss)){
			pMiss = 0.35;
			cout << "[OccMap]: No p miss. Use default: 0.35." << endl;
		}
		else{
			cout << "[OccMap]: P miss: " << pMiss << endl;
		}
		this->pMissLog_ = this->logit(pMiss);

		// p min
		double pMin;
		if (not this->nh_.getParam("occupancy_map/p_min", pMin)){
			pHit = 0.12;
			cout << "[OccMap]: No p min. Use default: 0.12." << endl;
		}
		else{
			cout << "[OccMap]: P min: " << pMin << endl;
		}
		this->pMinLog_ = this->logit(pMin);

		// p max
		double pMax;
		if (not this->nh_.getParam("occupancy_map/p_max", pMax)){
			pMax = 0.97;
			cout << "[OccMap]: No p max. Use default: 0.97." << endl;
		}
		else{
			cout << "[OccMap]: P max: " << pMax << endl;
		}
		this->pMaxLog_ = this->logit(pMax);

		// p occ
		double pOcc;
		if (not this->nh_.getParam("occupancy_map/p_occ", pOcc)){
			pOcc = 0.80;
			cout << "[OccMap]: No p occ. Use default: 0.80." << endl;
		}
		else{
			cout << "[OccMap]: P occ: " << pOcc << endl;
		}
		this->pOccLog_ = this->logit(pOcc);


		// map resolution
		if (not this->nh_.getParam("occupancy_map/map_resolution", this->mapRes_)){
			this->mapRes_ = 0.1;
			cout << "[OccMap]: No map resolution. Use default: 0.1." << endl;
		}
		else{
			cout << "[OccMap]: Map resolution: " << this->mapRes_ << endl;
		}

		// ground height
		if (not this->nh_.getParam("occupancy_map/ground_height", this->groundHeight_)){
			this->groundHeight_ = 0.0;
			cout << "[OccMap]: No ground height. Use default: 0.0." << endl;
		}
		else{
			cout << "[OccMap]: Ground height: " << this->groundHeight_ << endl;
		}


		// map size
		std::vector<double> mapSizeVec (3);
		if (not this->nh_.getParam("occupancy_map/map_size", mapSizeVec)){
			mapSizeVec[0] = 20; mapSizeVec[1] = 20; mapSizeVec[2] = 3;
			cout << "[OccMap]: No map size. Use default: [20, 20, 3]." << endl;
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
			this->occupancyInflated_.resize(reservedSize, 0);
			this->flagTraverse_.resize(reservedSize, -1);
			this->flagRayend_.resize(reservedSize, -1);

			cout << "[OccMap]: Map size: " << "[" << mapSizeVec[0] << ", " << mapSizeVec[1] << ", " << mapSizeVec[2] << "]" << endl;
		}

		// local update range
		std::vector<double> localUpdateRangeVec;
		if (not this->nh_.getParam("occupancy_map/local_update_range", localUpdateRangeVec)){
			localUpdateRangeVec = std::vector<double>{5.0, 5.0, 3.0};
			cout << "[OccMap]: No local update range. Use default: [5.0, 5.0, 3.0] m." << endl;
		}
		else{
			cout << "[OccMap]: Local update range: " << "[" << localUpdateRangeVec[0] << ", " << localUpdateRangeVec[1] << ", " << localUpdateRangeVec[2] << "]" << endl;
		}
		this->localUpdateRange_(0) = localUpdateRangeVec[0]; this->localUpdateRange_(1) = localUpdateRangeVec[1]; this->localUpdateRange_(2) = localUpdateRangeVec[2];


		// local map size
		std::vector<double> localMapSizeVec;
		if (not this->nh_.getParam("occupancy_map/local_map_size", localMapSizeVec)){
			localMapSizeVec = std::vector<double>{6.0, 6.0, 2.0};
			cout << "[OccMap]: No local map size. Use default: [6.0, 6.0, 2.0] m." << endl;
		}
		else{
			cout << "[OccMap]: Local map size: " << "[" << localMapSizeVec[0] << ", " << localMapSizeVec[1] << ", " << localMapSizeVec[2] << "]" << endl;
		}
		this->localMapSize_(0) = localUpdateRangeVec[0]; this->localMapSize_(1) = localUpdateRangeVec[1]; this->localMapSize_(2) = localUpdateRangeVec[2];
		this->localMapVoxel_(0) = int(ceil(localUpdateRangeVec[0]/this->mapRes_)); this->localMapVoxel_(1) = int(ceil(localUpdateRangeVec[1]/this->mapRes_)); this->localMapVoxel_(2) = int(ceil(localUpdateRangeVec[2]/this->mapRes_));

		// local bound inflate factor
		if (not this->nh_.getParam("occupancy_map/local_bound_inflation", this->localBoundInflate_)){
			this->localBoundInflate_ = 0.0;
			cout << "[OccMap]: No local bound inflate. Use default: 0.0 m." << endl;
		}
		else{
			cout << "[OccMap]: Local bound inflate: " << this->localBoundInflate_ << endl;
		}

		// max vis height
		if (not this->nh_.getParam("occupancy_map/max_height_visualization", this->maxVisHeight_)){
			this->maxVisHeight_ = 3.0;
			cout << "[OccMap]: No max visualization height. Use default: 3.0 m." << endl;
		}
		else{
			cout << "[OccMap]: Max visualization height: " << this->maxVisHeight_ << endl;
		}


	}

	void occMap::registerCallback(){
		// depth pose callback
		this->depthSub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(this->nh_, this->depthTopicName_, 50));
		this->poseSub_.reset(new message_filters::Subscriber<geometry_msgs::PoseStamped>(this->nh_, this->poseTopicName_, 25));
		this->depthPoseSync_.reset(new message_filters::Synchronizer<depthPoseSync>(depthPoseSync(100), *this->depthSub_, *this->poseSub_));
		this->depthPoseSync_->registerCallback(boost::bind(&occMap::depthPoseCB, this, _1, _2));

		// occupancy update callback
		this->occTimer_ = this->nh_.createTimer(ros::Duration(0.05), &occMap::updateOccupancyCB, this);

		// visualization callback
		this->visTimer_ = this->nh_.createTimer(ros::Duration(0.05), &occMap::visCB, this);
	}

	void occMap::registerPub(){
		this->depthCloudPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>("/occupancy_map/depth_cloud", 10);
		this->mapVisPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>("/occupancy_map/voxel_map", 10);
	}


	void occMap::depthPoseCB(const sensor_msgs::ImageConstPtr& img, const geometry_msgs::PoseStampedConstPtr& pose){
		// cout << "depth pose callback trigger" << endl;
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

		// TODO: check whether current pose is in map (if not we don't update occupancy map)
		if (this->isInMap(this->position_)){
			this->occNeedUpdate_ = true;
			// cout << "occupancy need update!!" << endl;
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

		// infalte map
		endTime = ros::Time::now();
		cout << "[OccMap]: Occupancy update time: " << (endTime - startTime).toSec() << " s." << endl;
		this->occNeedUpdate_ = false;
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


		 // this->projPointsNum_ = 0;
		 //  uint16_t* row_ptr;
		 //  // int cols = current_img_.cols, rows = current_img_.rows;
		 //  int cols = this->depthImage_.cols;
		 //  int rows = this->depthImage_.rows;
		 //  // cout << "cols: " << cols << " rows: " << rows << endl;

		 //  double depth;

		 //  Eigen::Matrix3d camera_r = this->orientation_;

		 //  // cout << "rotate: " << md_.camera_q_.toRotationMatrix() << endl;
		 //  // std::cout << "pos in proj: " << this->position_ << std::endl;

		 //  if (false) {
		 //    for (int v = 0; v < rows; v++) {
		 //      row_ptr = this->depthImage_.ptr<uint16_t>(v);

		 //    }
		 //  }
		 //  /* use depth filter */
		 //  else {

		 //    if (false){

		 //    }
		 //    else {
		 //      Eigen::Vector3d pt_cur, pt_world, pt_reproj;

		 //      Eigen::Matrix3d last_camera_r_inv;
		 //      last_camera_r_inv = this->orientation_;
		 //      const double inv_factor = 1.0 / this->depthScale_;

		 //      for (int v = this->depthFilterMargin_; v < rows - this->depthFilterMargin_; v += this->skipPixel_) {
		 //        row_ptr = this->depthImage_.ptr<uint16_t>(v) + this->depthFilterMargin_;

		 //        for (int u = this->depthFilterMargin_; u < cols - this->depthFilterMargin_;
		 //             u += this->skipPixel_) {

		 //          depth = (*row_ptr) * inv_factor;
		 //          row_ptr = row_ptr + this->skipPixel_;

		 //          // filter depth
		 //          // depth += rand_noise_(eng_);
		 //          // if (depth > 0.01) depth += rand_noise2_(eng_);

		 //          if (*row_ptr == 0) {
		 //            depth = this->raycastMaxLength_ + 0.1;
		 //          } else if (depth < this->depthMinValue_) {
		 //            continue;
		 //          } else if (depth > this->depthMaxValue_) {
		 //            depth = this->raycastMaxLength_ + 0.1;
		 //          }

		 //          // project to world frame
		 //          pt_cur(0) = (u - this->cx_) * depth / this->fx_;
		 //          pt_cur(1) = (v - this->cy_) * depth / this->fy_;
		 //          pt_cur(2) = depth;

		 //          // pt_cur(0) = depth;
		 //          // pt_cur(1) = -(u - mp_.cx_) * depth / mp_.fx_;
		 //          // pt_cur(2) = -(v - mp_.cy_) * depth / mp_.fy_;

		 //          pt_world = camera_r * pt_cur + this->position_;
		 //          // if (!isInMap(pt_world)) {
		 //          //   pt_world = closetPointInMap(pt_world, this->position_);
		 //          // }

		 //          this->projPoints_[this->projPointsNum_++] = pt_world;
		 //          // cout << "point: " << pt_world << endl;

		 //          // check consistency with last image, disabled...

		 //        }
		 //      }
		 //    }
		 //  }
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

			// if (hit == 0 and miss == 0){
			// 	continue; // this has been updated. repeated points
			// }

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

		// =======================================================================================================================================
		// if (this->projPointsNum_ == 0) return;

		// ros::Time t1, t2;

		// this->raycastNum_ += 1;

		// int vox_idx;
		// double length;

		// // bounding box of updated region
		// double min_x = this->mapVoxelMax_(0);
		// double min_y = this->mapVoxelMax_(1);
		// double min_z = this->mapVoxelMax_(2);

		// double max_x = this->mapVoxelMin_(0);
		// double max_y = this->mapVoxelMin_(1);
		// double max_z = this->mapVoxelMin_(2);

		// RayCaster raycaster;
		// Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
		// Eigen::Vector3d ray_pt, pt_w;

		// for (int i = 0; i < this->projPointsNum_; ++i) {
		// pt_w = this->projPoints_[i];

		// // set flag for projected point

		// if (!isInMap(pt_w)) {
		//   pt_w = closetPointInMap(pt_w, this->position_);

		//   length = (pt_w - this->position_).norm();
		//   if (length > this->raycastMaxLength_) {
		//     pt_w = (pt_w - this->position_) / length * this->raycastMaxLength_ + this->position_;
		//   }
		//   vox_idx = setCacheOccupancy(pt_w, 0);

		// } else {
		//   length = (pt_w - this->position_).norm();

		//   if (length > this->raycastMaxLength_) {
		//     pt_w = (pt_w - this->position_) / length * this->raycastMaxLength_ + this->position_;
		//     vox_idx = setCacheOccupancy(pt_w, 0);
		//   } else {
		//     vox_idx = setCacheOccupancy(pt_w, 1);
		//   }
		// }

		// max_x = std::max(max_x, pt_w(0));
		// max_y = std::max(max_y, pt_w(1));
		// max_z = std::max(max_z, pt_w(2));

		// min_x = std::min(min_x, pt_w(0));
		// min_y = std::min(min_y, pt_w(1));
		// min_z = std::min(min_z, pt_w(2));

		// // raycasting between camera center and point

		//   if (this->flagRayend_[vox_idx] == this->raycastNum_) {
		//     continue;
		//   } else {
		//     this->flagRayend_[vox_idx] = this->raycastNum_;
		//   }
		

		// raycaster.setInput(pt_w / this->mapRes_, this->position_ / this->mapRes_);

		// while (raycaster.step(ray_pt)) {
		//   Eigen::Vector3d tmp = (ray_pt + half) * this->mapRes_;
		//   length = (tmp - this->position_).norm();

		//   // if (length < mp_.min_ray_length_) break;

		//   vox_idx = setCacheOccupancy(tmp, 0);

		//     if (this->flagTraverse_[vox_idx] == this->raycastNum_) {
		//       break;
		//     } else {
		//       this->flagTraverse_[vox_idx] = this->raycastNum_;
		//     }
		  
		// }
		// }

		// // determine the local bounding box for updating ESDF
		// min_x = std::min(min_x, this->position_(0));
		// min_y = std::min(min_y, this->position_(1));
		// min_z = std::min(min_z, this->position_(2));

		// max_x = std::max(max_x, this->position_(0));
		// max_y = std::max(max_y, this->position_(1));
		// max_z = std::max(max_z, this->position_(2));
		// max_z = std::max(max_z, this->groundHeight_);

		// posToIndex(Eigen::Vector3d(max_x, max_y, max_z), this->localBoundMax_);
		// posToIndex(Eigen::Vector3d(min_x, min_y, min_z), this->localBoundMin_);

		// int esdf_inf = ceil(this->localBoundInflate_ / this->mapRes_);
		// this->localBoundMax_ += esdf_inf * Eigen::Vector3i(1, 1, 0);
		// this->localBoundMin_ -= esdf_inf * Eigen::Vector3i(1, 1, 0);
		// boundIndex(this->localBoundMax_);
		// boundIndex(this->localBoundMin_);

		// // this-> = true;

		// // update occupancy cached in queue
		// Eigen::Vector3d local_range_min = this->position_ - this->localUpdateRange_;
		// Eigen::Vector3d local_range_max = this->position_ + this->localUpdateRange_;

		// Eigen::Vector3i min_id, max_id;
		// posToIndex(local_range_min, min_id);
		// posToIndex(local_range_max, max_id);
		// boundIndex(min_id);
		// boundIndex(max_id);

		// // std::cout << "cache all: " << this->updateVoxelCache_.size() << std::endl;

		// while (!this->updateVoxelCache_.empty()) {

		// Eigen::Vector3i idx = this->updateVoxelCache_.front();
		// int idx_ctns = indexToAddress(idx);
		// this->updateVoxelCache_.pop();

		// double log_odds_update =
		//     this->countHit_[idx_ctns] >= this->countHitMiss_[idx_ctns] - this->countHit_[idx_ctns] ?
		//     this->pHitLog_ :
		//     this->pMissLog_;

		// this->countHit_[idx_ctns] = this->countHitMiss_[idx_ctns] = 0;

		// if (log_odds_update >= 0 && this->occupancy_[idx_ctns] >= this->pMaxLog_) {
		//   continue;
		// } else if (log_odds_update <= 0 && this->occupancy_[idx_ctns] <= this->pMinLog_) {
		//   this->occupancy_[idx_ctns] = this->pMinLog_;
		//   continue;
		// }

		// bool in_local = idx(0) >= min_id(0) && idx(0) <= max_id(0) && idx(1) >= min_id(1) &&
		//     idx(1) <= max_id(1) && idx(2) >= min_id(2) && idx(2) <= max_id(2);
		// if (!in_local) {
		//   this->occupancy_[idx_ctns] = this->pMinLog_;
		// }

		// this->occupancy_[idx_ctns] =
		//     std::min(std::max(this->occupancy_[idx_ctns] + log_odds_update, this->pMinLog_),
		//              this->pMaxLog_);
		// // cout << "update occupancy" << std::min(std::max(this->occupancy_[idx_ctns] + log_odds_update, this->pMinLog_),
		// //              this->pMaxLog_) << endl;

		// }
	}

	void occMap::cleanLocalMap(){

	}

	void occMap::inflateLocalMap(){

	}

	bool occMap::isOccupied(const Eigen::Vector3d& pos){
		Eigen::Vector3i idx;
		this->posToIndex(pos, idx);
		return this->isOccupied(idx);
	}

	bool occMap::isOccupied(const Eigen::Vector3i& idx){
		int address = this->indexToAddress(idx);
		return this->occupancy_[address] >= this->pOccLog_;
	}

	bool occMap::isFree(const Eigen::Vector3d& pos){
		Eigen::Vector3i idx;
		this->posToIndex(pos, idx);
		return this->isFree(idx);
	}

	bool occMap::isFree(const Eigen::Vector3i& idx){
		int address = this->indexToAddress(idx);
		return (this->occupancy_[address] < this->pOccLog_) and (this->occupancy_[address] >= this->pMinLog_);
	}

	bool occMap::isUnknown(const Eigen::Vector3d& pos){
		Eigen::Vector3i idx;
		this->posToIndex(pos, idx);
		return this->isUnknown(idx);
	}

	bool occMap::isUnknown(const Eigen::Vector3i& idx){
		int address = this->indexToAddress(idx);
		return this->occupancy_[address] < this->pMinLog_;		
	}

	void occMap::visCB(const ros::TimerEvent& ){
		this->publishProjPoints();
		this->publishMap();
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

		Eigen::Vector3d minRange = this->position_ - localMapSize_;
		Eigen::Vector3d maxRange = this->position_ + localMapSize_;
		Eigen::Vector3i minRangeIdx, maxRangeIdx;
		this->posToIndex(minRange, minRangeIdx);
		this->posToIndex(maxRange, maxRangeIdx);
		this->boundIndex(minRangeIdx);
		this->boundIndex(maxRangeIdx);

		for (int x=minRangeIdx(0); x<maxRangeIdx(0); ++x){
			for (int y=minRangeIdx(1); y<maxRangeIdx(1); ++y){
				for (int z=minRangeIdx(2); z<maxRangeIdx(2); ++z){
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


	bool occMap::isInMap(const Eigen::Vector3d& pos){
		if ((pos(0) >= this->mapSizeMin_(0)) and (pos(0) <= this->mapSizeMax_(0)) and 
			(pos(1) >= this->mapSizeMin_(1)) and (pos(1) <= this->mapSizeMax_(1)) and 
			(pos(2) >= this->mapSizeMin_(2)) and (pos(2) <= this->mapSizeMax_(2))){
			return true;
		}
		else{
			return false;
		}
	}

	bool occMap::isInMap(const Eigen::Vector3i& idx){
		if ((idx(0) >= this->mapVoxelMin_(0)) and (idx(0) <= this->mapVoxelMax_(0)) and
		    (idx(1) >= this->mapVoxelMin_(1)) and (idx(1) <= this->mapVoxelMax_(1)) and 
		    (idx(2) >= this->mapVoxelMin_(2)) and (idx(2) <= this->mapVoxelMax_(2))){
			return true;
		}
		else{
			return false;
		}
	}

	void occMap::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& idx){
		idx(0) = floor( (pos(0) - this->mapSizeMin_(0) ) / this->mapRes_ );
		idx(1) = floor( (pos(1) - this->mapSizeMin_(1) ) / this->mapRes_ );
		idx(2) = floor( (pos(2) - this->mapSizeMin_(2) ) / this->mapRes_ );
	}

	void occMap::indexToPos(const Eigen::Vector3i& idx, Eigen::Vector3d& pos){
		pos(0) = (idx(0) + 0.5) * this->mapRes_ + this->mapSizeMin_(0); 
		pos(1) = (idx(1) + 0.5) * this->mapRes_ + this->mapSizeMin_(1);
		pos(2) = (idx(2) + 0.5) * this->mapRes_ + this->mapSizeMin_(2);
	}

	int occMap::posToAddress(const Eigen::Vector3d& pos){
		Eigen::Vector3i idx;
		this->posToIndex(pos, idx);
		return this->indexToAddress(idx);
	}

	int occMap::indexToAddress(const Eigen::Vector3i& idx){
		return idx(0) * this->mapVoxelMax_(1) * this->mapVoxelMax_(2) + idx(1) * this->mapVoxelMax_(2) + idx(2);
	}

	void occMap::boundIndex(Eigen::Vector3i& idx){
		Eigen::Vector3i temp;
		temp(0) = std::max(std::min(idx(0), this->mapVoxelMax_(0)), this->mapVoxelMin_(0));
		temp(1) = std::max(std::min(idx(1), this->mapVoxelMax_(1)), this->mapVoxelMin_(1));
		temp(2) = std::max(std::min(idx(2), this->mapVoxelMax_(2)), this->mapVoxelMin_(2));
		idx = temp;
	}

	bool occMap::isInLocalUpdateRange(const Eigen::Vector3d& pos){
		Eigen::Vector3i idx;
		this->posToIndex(pos, idx);
		return this->isInLocalUpdateRange(idx);
	}

	bool occMap::isInLocalUpdateRange(const Eigen::Vector3i& idx){
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

	Eigen::Vector3d occMap::adjustPointInMap(const Eigen::Vector3d& point){
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


	Eigen::Vector3d occMap::adjustPointRayLength(const Eigen::Vector3d& point){
		double length = (point - this->position_).norm();
		return (point - this->position_) * (this->raycastMaxLength_/length) + this->position_;
	}

	int occMap::updateOccupancyInfo(const Eigen::Vector3d& point, bool isOccupied){
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

	void occMap::getCameraPose(const geometry_msgs::PoseStampedConstPtr& pose, Eigen::Matrix4d& camPoseMatrix){
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

	int occMap::setCacheOccupancy(Eigen::Vector3d pos, int occ) {

	  Eigen::Vector3i id;
	  posToIndex(pos, id);
	  int idx_ctns = indexToAddress(id);

	  this->countHitMiss_[idx_ctns] += 1;

	  if (this->countHitMiss_[idx_ctns] == 1) {
	    this->updateVoxelCache_.push(id);
	  }

	  if (occ == 1) this->countHit_[idx_ctns] += 1;

	  return idx_ctns;
	}

	Eigen::Vector3d occMap::closetPointInMap(const Eigen::Vector3d& pt, const Eigen::Vector3d& camera_pt) {
		  Eigen::Vector3d diff = pt - camera_pt;
		  Eigen::Vector3d max_tc = this->mapSizeMax_ - camera_pt;
		  Eigen::Vector3d min_tc = this->mapSizeMin_ - camera_pt;

		  double min_t = 1000000;

		  for (int i = 0; i < 3; ++i) {
		    if (fabs(diff[i]) > 0) {

		      double t1 = max_tc[i] / diff[i];
		      if (t1 > 0 && t1 < min_t) min_t = t1;

		      double t2 = min_tc[i] / diff[i];
		      if (t2 > 0 && t2 < min_t) min_t = t2;
		    }
		  }

		  return camera_pt + (min_t - 1e-3) * diff;
		}

}