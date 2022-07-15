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

		// map resolution
		if (not this->nh_.getParam("occupancy_map/map_resolution", this->mapRes_)){
			this->mapRes_ = 0.1;
			cout << "[OccMap]: No map resolution. Use default: 0.1." << endl;
		}
		else{
			cout << "[OccMap]: Map resolution: " << this->mapRes_ << endl;
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
			this->mapSizeMin_(2) = -mapSizeVec[2]/2; this->mapSizeMax_(2) = mapSizeVec[2]/2;
			
			// min max for voxel
			this->mapVoxelMin_(0) = 0; this->mapVoxelMin_(0) = ceil(mapSizeVec[0]/this->mapRes_);
			this->mapVoxelMin_(1) = 0; this->mapVoxelMin_(1) = ceil(mapSizeVec[1]/this->mapRes_);
			this->mapVoxelMin_(2) = 0; this->mapVoxelMin_(2) = ceil(mapSizeVec[2]/this->mapRes_);

			cout << "[OccMap]: Map size: " << "[" << mapSizeVec[0] << ", " << mapSizeVec[1] << ", " << mapSizeVec[2] << "]" << endl;
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

		// clear local map

		// infalte map
		endTime = ros::Time::now();
		cout << "[OccMap]: Occupancy update time: " << (endTime - startTime).toSec() << " s." << endl;
		this->occNeedUpdate_ = false;
	}


	void occMap::projectDepthImage(){
		int projPointsNum = 0;

		int cols = this->depthImage_.cols;
		int rows = this->depthImage_.rows;


		Eigen::Vector3d currPointCam, currPointMap;
		double depth;

		// iterate through each pixel in the depth image
		for (int v=this->depthFilterMargin_; v<rows-this->depthFilterMargin_; v+=this->skipPixel_){ // row
			for (int u=this->depthFilterMargin_; u<cols-this->depthFilterMargin_; u+=this->skipPixel_){ // column
				depth = this->depthImage_.at<uint16_t>(v, u) / this->depthScale_;

				// // filter depth value
				if (depth == 0 or depth > this->depthMaxValue_){
					depth = this->raycastMaxLength_ + 0.1; // dummy point for raycasting
				}
				else if (depth < this->depthMinValue_){
					continue; // ignore too close points
				}

				// // // // get 3D point in camera frame
				currPointCam(0) = (u - this->cx_) * depth/this->fx_;
				currPointCam(1) = (v - this->cy_) * depth/this->fy_;
				currPointCam(2) = depth;
				currPointMap = this->orientation_ * currPointCam + this->position_; // transform to map coordinate
				// // // // store current point
				this->projPoints_[projPointsNum++] = currPointMap;
			}
		}
		this->projPointsNum_ = projPointsNum;
	}


	void occMap::raycastUpdate(){

	}

	void occMap::cleanLocalMap(){

	}

	void occMap::inflateLocalMap(){

	}

	void occMap::visCB(const ros::TimerEvent& ){
		// this->publishProjPoints();
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

}