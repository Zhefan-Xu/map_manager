/*
	FILE: occupancyMap.cpp
	--------------------------------------
	function definition of occupancy map
*/
#include <map_manager/multiRoboOccupancyMap.h>

namespace mapManager{
	multiRoboOccMap::multiRoboOccMap(){
		this->ns_ = "multi_robo_occupancy_map";
		this->hint_ = "[multiRoboOccMap]";
	}

	multiRoboOccMap::multiRoboOccMap(const ros::NodeHandle& nh) : nh_(nh){
		this->ns_ = "multi_robo_occupancy_map";
		this->hint_ = "[multiRoboOccMap]";
		this->initMultiRoboParam();
		this->initParam();
		this->initPrebuiltMap();
		this->registerPub();
		this->registerCallback();
	}

	void multiRoboOccMap::initMultiRoboOccMap(const ros::NodeHandle& nh){
		this->nh_ = nh;
		this->initMultiRoboParam();
		this->initParam();
		this->initPrebuiltMap();
		this->registerPub();
		this->registerMultiRoboPub();
		this->registerMultiRoboCallback();
	}

	void multiRoboOccMap::initMultiRoboParam(){
		// robot num
		if (not this->nh_.getParam(this->ns_ + "/robot_num", this->robot_num_)){
			this->robot_num_ = 0;
			cout << this->hint_ << ": No robot num option. Use default: 0" << endl;
		}
		else{
			cout << this->hint_ << ": robot num: " << this->robot_num_ << endl;
		}	
		this->allRobotsReady_ = false;

		// robot id
		if (not this->nh_.getParam(this->ns_ + "/robot_id", this->robot_id_)){
			this->robot_id_ = 0;
			cout << this->hint_ << ": No robot id option. Use default: 0" << endl;
		}
		else{
			cout << this->hint_ << ": robot id: " << this->robot_id_ << endl;
		}	
		this->sharedVoxels_.from_id = this->robot_id_;

		// transform matrix: map to global(for multi-robot map transmission)
		std::vector<double> global2MapVec (16);
		if (not this->nh_.getParam(this->ns_ + "/global_to_map_" + std::to_string(this->robot_id_), global2MapVec)){
			ROS_ERROR("[multiRoboOccMap]: Please check global to map matrix!");
		}
		else{
			for (int i=0; i<4; ++i){
				for (int j=0; j<4; ++j){
					this->global2Map_(i, j) = global2MapVec[i * 4 + j];
				}
			}
			cout << this->hint_ << ": from global to map: " << endl;
			cout << this->global2Map_ << endl;
		}


	}

	void multiRoboOccMap::registerMultiRoboCallback(){

		if (this->sensorInputMode_ == 0){
			// depth pose callback
			this->depthSub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(this->nh_, this->depthTopicName_, 50));
			if (this->localizationMode_ == 0){
				this->poseSub_.reset(new message_filters::Subscriber<geometry_msgs::PoseStamped>(this->nh_, this->poseTopicName_, 25));
				this->depthPoseSync_.reset(new message_filters::Synchronizer<depthPoseSync>(depthPoseSync(100), *this->depthSub_, *this->poseSub_));
				this->depthPoseSync_->registerCallback(boost::bind(&multiRoboOccMap::depthPoseCB, this, _1, _2));
			}
			else if (this->localizationMode_ == 1){
				this->odomSub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(this->nh_, this->odomTopicName_, 25));
				this->depthOdomSync_.reset(new message_filters::Synchronizer<depthOdomSync>(depthOdomSync(100), *this->depthSub_, *this->odomSub_));
				this->depthOdomSync_->registerCallback(boost::bind(&multiRoboOccMap::depthOdomCB, this, _1, _2));
			}
			else{
				ROS_ERROR("[OccMap]: Invalid localization mode!");
				exit(0);
			}
		}
		else if (this->sensorInputMode_ == 1){
			// pointcloud callback
			this->pointcloudSub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(this->nh_, this->pointcloudTopicName_, 50));
			if (this->localizationMode_ == 0){
				this->poseSub_.reset(new message_filters::Subscriber<geometry_msgs::PoseStamped>(this->nh_, this->poseTopicName_, 25));
				this->pointcloudPoseSync_.reset(new message_filters::Synchronizer<pointcloudPoseSync>(pointcloudPoseSync(100), *this->pointcloudSub_, *this->poseSub_));
				this->pointcloudPoseSync_->registerCallback(boost::bind(&multiRoboOccMap::pointcloudPoseCB, this, _1, _2));
			}
			else if (this->localizationMode_ == 1){
				this->odomSub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(this->nh_, this->odomTopicName_, 25));
				this->pointcloudOdomSync_.reset(new message_filters::Synchronizer<pointcloudOdomSync>(pointcloudOdomSync(100), *this->pointcloudSub_, *this->odomSub_));
				this->pointcloudOdomSync_->registerCallback(boost::bind(&multiRoboOccMap::pointcloudOdomCB, this, _1, _2));
			}
			else{
				ROS_ERROR("[OccMap]: Invalid localization mode!");
				exit(0);
			}
		}
		else{
			ROS_ERROR("[OccMap]: Invalid sensor input mode!");
			exit(0);
		}

		// occupancy update callback
		this->occTimer_ = this->nh_.createTimer(ros::Duration(0.05), &multiRoboOccMap::updateOccupancyCB, this);

		// map inflation callback
		this->inflateTimer_ = this->nh_.createTimer(ros::Duration(0.05), &occMap::inflateMapCB, (occMap*)this);

		// visualization callback
		this->visTimer_ = this->nh_.createTimer(ros::Duration(0.05), &occMap::visCB, (occMap*)this);
		// visualization callback
		this->mapSharedPubTimer_ = this->nh_.createTimer(ros::Duration(0.05), &multiRoboOccMap::mapSharedPubCB, this);

		// subscrived map shared by other robots
		this->mapSharedSub_ = this->nh_.subscribe(this->ns_ + "/shared_map", 10, &multiRoboOccMap::mapSharedSubCB, this);
		
		// subscribe all robots states in the robot network
		this->robotStatesSub_ = this->nh_.subscribe(this->ns_ + "/robot_states", 10, &multiRoboOccMap::robotStatesSubCB, this);
	}

	void multiRoboOccMap::depthPoseCB(const sensor_msgs::ImageConstPtr& img, const geometry_msgs::PoseStampedConstPtr& pose){
		// store current depth image
		cv_bridge::CvImagePtr imgPtr = cv_bridge::toCvCopy(img, img->encoding);
		if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1){
			(imgPtr->image).convertTo(imgPtr->image, CV_16UC1, this->depthScale_);
		}
		imgPtr->image.copyTo(this->depthImage_);

		// store current position and orientation (camera)
		Eigen::Matrix4d camPoseMatrix;
		this->getCameraPose(pose, camPoseMatrix);
		camPoseMatrix = this->global2Map_ * camPoseMatrix; // transform to global map frame according to inter_robot transform

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

	void multiRoboOccMap::depthOdomCB(const sensor_msgs::ImageConstPtr& img, const nav_msgs::OdometryConstPtr& odom){
		// store current depth image
		cv_bridge::CvImagePtr imgPtr = cv_bridge::toCvCopy(img, img->encoding);
		if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1){
			(imgPtr->image).convertTo(imgPtr->image, CV_16UC1, this->depthScale_);
		}
		imgPtr->image.copyTo(this->depthImage_);

		// store current position and orientation (camera)
		Eigen::Matrix4d camPoseMatrix;
		this->getCameraPose(odom, camPoseMatrix);
		// camPoseMatrix = this->global2Map_ * camPoseMatrix; // transform to global map frame according to inter_robot transform

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

	void multiRoboOccMap::pointcloudPoseCB(const sensor_msgs::PointCloud2ConstPtr& pointcloud, const geometry_msgs::PoseStampedConstPtr& pose){
		// directly get the point cloud
		pcl::PCLPointCloud2 pclPC2;
		pcl_conversions::toPCL(*pointcloud, pclPC2); // convert ros pointcloud2 to pcl pointcloud2
		pcl::fromPCLPointCloud2(pclPC2, this->pointcloud_);

		// store current position and orientation (camera)
		Eigen::Matrix4d camPoseMatrix;
		this->getCameraPose(pose, camPoseMatrix);
		// camPoseMatrix = this->global2Map_ * camPoseMatrix; // transform to global map frame according to inter_robot transform

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

	void multiRoboOccMap::pointcloudOdomCB(const sensor_msgs::PointCloud2ConstPtr& pointcloud, const nav_msgs::OdometryConstPtr& odom){
		// directly get the point cloud
		pcl::PCLPointCloud2 pclPC2;
		pcl_conversions::toPCL(*pointcloud, pclPC2); // convert ros pointcloud2 to pcl pointcloud2
		pcl::fromPCLPointCloud2(pclPC2, this->pointcloud_);


		// store current position and orientation (camera)
		Eigen::Matrix4d camPoseMatrix;
		this->getCameraPose(odom, camPoseMatrix);
		camPoseMatrix = this->global2Map_ * camPoseMatrix; // transform to global map frame according to inter_robot transform

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

	void multiRoboOccMap::registerMultiRoboPub(){
		
		// map_manager::sharedVoxels test;
		this->mapSharedPub_ = this->nh_.advertise<map_manager::sharedVoxels>(this->ns_ + "/shared_map", 10);

		// publish current robot's states to the robot network
		this->robotStatesPub_ = this->nh_.advertise<map_manager::robotStates>(this->ns_ + "/robot_states", 10);
	}

	void multiRoboOccMap::mapSharedSubCB(const map_manager::sharedVoxelsConstPtr& incomeVoxels){
		// tell others current robot is ready
		map_manager::robotStates robotStatesMsg;
		robotStatesMsg.robot_id = this->robot_id_;
		robotStatesMsg.ready = true;
		this->robotStatesPub_.publish(robotStatesMsg);
		// Do not merge map published by self
		if (this->robot_id_ == incomeVoxels->from_id){
			return;
		}

		for (size_t i=0 ; i<incomeVoxels->occupancy.size() ; ++i){
			int ind = this->posToAddress(incomeVoxels->positions[i].x, 
										 incomeVoxels->positions[i].y, 
										 incomeVoxels->positions[i].z);
			if (incomeVoxels->occupancy[i]){
				this->occupancy_[ind] = this->pMaxLog_; 
			}
			else{
				this->occupancy_[ind] = this->pMinLog_;
			}
		}
	}

	void multiRoboOccMap::mapSharedPubCB(const ros::TimerEvent& ){
		this->mapSharedPub_.publish(this->sharedVoxels_);

		// all others are ready to subscribe new map, so we can delete prevoius shared voxels
		if (this->allRobotsReady_){
			this->sharedVoxels_.occupancy.clear();
			this->sharedVoxels_.positions.clear();
		}
	}

	void multiRoboOccMap::robotStatesSubCB(const map_manager::robotStatesConstPtr& states){
		if (states->ready){
			// push robot id if it is a new robot
			if (this->readyRobotsID_.empty()){
				this->readyRobotsID_.push_back(states->robot_id);
			}
			else if (std::find(this->readyRobotsID_.begin(), this->readyRobotsID_.end(), states->robot_id)==this->readyRobotsID_.end() 
						  and this->readyRobotsID_.back()!=states->robot_id){
				this->readyRobotsID_.push_back(states->robot_id);
			}
			if (this->readyRobotsID_.size() == this->robot_num_){
				this->allRobotsReady_ = true;
			}
		}
	}

	void multiRoboOccMap::updateOccupancyCB(const ros::TimerEvent& ){
		if (not this->occNeedUpdate_){
			return;
		}
		// cout << "update occupancy map" << endl;
		ros::Time startTime, endTime;
		
		startTime = ros::Time::now();
		if (this->sensorInputMode_ == 0){
			// project 3D points from depth map
			this->projectDepthImage();
		}
		else if (this->sensorInputMode_ == 1){
			// directly get pointcloud
			this->getPointcloud();
		}

		// raycasting and update occupancy
		this->raycastUpdate();


		// clear local map
		if (this->cleanLocalMap_){
			this->cleanLocalMap();
		}
		
		// infalte map
		// this->inflateLocalMap();
		endTime = ros::Time::now();
		if (this->verbose_){
			cout << this->hint_ << ": Occupancy update time: " << (endTime - startTime).toSec() << " s." << endl;
		}
		this->occNeedUpdate_ = false;
		this->mapNeedInflate_ = true;
	}

	void multiRoboOccMap::raycastUpdate(){
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
			if (std::isnan(currPoint(0)) or std::isnan(currPoint(1)) or std::isnan(currPoint(2))){
				continue; // nan points can happen when we are using pointcloud as input
			}

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

			if (hit >= miss and hit != 0){
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

			if (this->useFreeRegions_){ // current used in simulation, this region will not be updated and directly set to free
				Eigen::Vector3d pos;
				this->indexToPos(cacheIdx, pos);
				if (this->isInHistFreeRegions(pos)){
					this->occupancy_[cacheAddress] = this->pMinLog_;
					continue;
				}
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

			double prevProb = this->occupancy_[cacheAddress];

			this->occupancy_[cacheAddress] = std::min(std::max(this->occupancy_[cacheAddress]+logUpdateValue, this->pMinLog_), this->pMaxLog_);

			// collect voxels whose status changed
			Eigen::Vector3d sharedVoxelPos;
			bool occupancy;
			geometry_msgs::Point p;
			this->indexToPos(cacheIdx,sharedVoxelPos);
			p.x = sharedVoxelPos(0);
			p.y = sharedVoxelPos(1);
			p.z = sharedVoxelPos(2);
			if (this->isOccupied(cacheIdx)){
				occupancy = true;
				// from unkown/free to occupied
				if (prevProb < this->pOccLog_){
					this->sharedVoxels_.occupancy.push_back(occupancy);
					this->sharedVoxels_.positions.push_back(p);
				}
			}
			else if(this->isFree(cacheIdx)){
				// from unkown/occupied to free
				occupancy = false;
				if (prevProb < this->pMinLog_ or prevProb >= this->pOccLog_){
					this->sharedVoxels_.occupancy.push_back(occupancy);
					this->sharedVoxels_.positions.push_back(p);
				}
			}


			// update the entire map range (if it is not unknown)
			if (not this->isUnknown(cacheIdx)){
				Eigen::Vector3d cachePos;
				this->indexToPos(cacheIdx, cachePos);
				if (cachePos(0) > this->currMapRangeMax_(0)){
					this->currMapRangeMax_(0) = cachePos(0);
				}
				else if (cachePos(0) < this->currMapRangeMin_(0)){
					this->currMapRangeMin_(0) = cachePos(0);
				}

				if (cachePos(1) > this->currMapRangeMax_(1)){
					this->currMapRangeMax_(1) = cachePos(1);
				}
				else if (cachePos(1) < this->currMapRangeMin_(1)){
					this->currMapRangeMin_(1) = cachePos(1);
				}

				if (cachePos(2) > this->currMapRangeMax_(2)){
					this->currMapRangeMax_(2) = cachePos(2);
				}
				else if (cachePos(2) < this->currMapRangeMin_(2)){
					this->currMapRangeMin_(2) = cachePos(2);
				}
			}
		}

	}
}
