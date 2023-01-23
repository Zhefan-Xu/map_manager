/*
    FILE: dynamicDetector.cpp
    ---------------------------------
    function implementation of dynamic osbtacle detector
*/
#include <map_manager/detector/dynamicDetector.h>


namespace mapManager{
    dynamicDetector::dynamicDetector(){
        this->ns_ = "dynamic_detector";
        this->hint_ = "[dynamicDetector]";
    }

    dynamicDetector::dynamicDetector(const ros::NodeHandle& nh){
        this->ns_ = "dynamic_detector";
        this->hint_ = "[dynamicDetector]";
        this->nh_ = nh;
        this->initParam();
        this->registerCallback();
    }

    void dynamicDetector::initDetector(const ros::NodeHandle& nh){
        this->nh_ = nh;
        this->initParam();
        this->registerCallback();
    }

    void dynamicDetector::initParam(){
        // localization mode
        if (not this->nh_.getParam(this->ns_ + "localization_mode", this->localizationMode_)){
            this->localizationMode_ = 0;
            cout << this->hint_ << ": No localization mode option. Use default: pose" << endl;
        }
        else{
            cout << this->hint_ << ": Localizaiton mode: pose (0)/odom (1). Your option: " << this->localizationMode_ << endl;
        }   

        // depth topic name
        if (not this->nh_.getParam(this->ns_ + "depth_image_topic", this->depthTopicName_)){
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
        }

        // Raycast max length
        if (not this->nh_.getParam(this->ns_ + "/raycast_max_length", this->raycastMaxLength_)){
            this->raycastMaxLength_ = 5.0;
            cout << this->hint_ << ": No raycast max length. Use default: 5.0." << endl;
        }
        else{
            cout << this->hint_ << ": Raycast max length: " << this->raycastMaxLength_ << endl;
        }

        // ground height
        if (not this->nh_.getParam(this->ns_ + "/ground_height", this->groundHeight_)){
            this->groundHeight_ = 0.1;
            std::cout << this->hint_ << ": No ground height parameter. Use default: 0.1m." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Ground height is set to: " << this->groundHeight_ << std::endl;
        }

        // minimum number of points in each cluster
        if (not this->nh_.getParam(this->ns_ + "/dbscan_min_points_cluster", this->dbMinPointsCluster_)){
            this->dbMinPointsCluster_ = 18;
            std::cout << this->hint_ << ": No DBSCAN minimum point in each cluster parameter. Use default: 18." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": DBSCAN Minimum point in each cluster is set to: " << this->dbMinPointsCluster_ << std::endl;
        }

        // search range
        if (not this->nh_.getParam(this->ns_ + "/dbscan_search_range_epsilon", this->dbEpsilon_)){
            this->dbEpsilon_ = 0.3;
            std::cout << this->hint_ << ": No DBSCAN epsilon parameter. Use default: 0.3." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": DBSCAN epsilon is set to: " << this->dbEpsilon_ << std::endl;
        }  
    }

    void dynamicDetector::registerCallback(){
        // depth pose callback
        this->depthSub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(this->nh_, this->depthTopicName_, 50));
        if (this->localizationMode_ == 0){
            this->poseSub_.reset(new message_filters::Subscriber<geometry_msgs::PoseStamped>(this->nh_, this->poseTopicName_, 25));
            this->depthPoseSync_.reset(new message_filters::Synchronizer<depthPoseSync>(depthPoseSync(100), *this->depthSub_, *this->poseSub_));
            this->depthPoseSync_->registerCallback(boost::bind(&dynamicDetector::depthPoseCB, this, _1, _2));
        }
        else if (this->localizationMode_ == 1){
            this->odomSub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(this->nh_, this->odomTopicName_, 25));
            this->depthOdomSync_.reset(new message_filters::Synchronizer<depthOdomSync>(depthOdomSync(100), *this->depthSub_, *this->odomSub_));
            this->depthOdomSync_->registerCallback(boost::bind(&dynamicDetector::depthOdomCB, this, _1, _2));
        }
        else{
            ROS_ERROR("[dynamicDetector]: Invalid localization mode!");
            exit(0);
        }

        // detection timer
        this->detectionTimer_ = this->nh_.createTimer(ros::Duration(0.033), &dynamicDetector::detectionCB, this);

        // tracking timer
        this->trackingTimer_ = this->nh_.createTimer(ros::Duration(0.033), &dynamicDetector::trackingCB, this);

        // classification timer
        this->classificationTimer_ = this->nh_.createTimer(ros::Duration(0.033), &dynamicDetector::classificationCB, this);
    }


    void dynamicDetector::depthPoseCB(const sensor_msgs::ImageConstPtr& img, const geometry_msgs::PoseStampedConstPtr& pose){
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
    }

    void dynamicDetector::depthOdomCB(const sensor_msgs::ImageConstPtr& img, const nav_msgs::OdometryConstPtr& odom){
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
    }


    void dynamicDetector::detectionCB(const ros::TimerEvent&){
        cout << "detector CB" << endl;
    }

    void dynamicDetector::trackingCB(const ros::TimerEvent&){
        cout << "tracking CB Not implemented yet." << endl;
        // data association


        // kalman filter tracking

    }

    void dynamicDetector::classificationCB(const ros::TimerEvent&){
        cout << "classification CB not implemented yet." << endl;
    }

    void dynamicDetector::uvDetect(std::vector<mapManager::box3D>& bboxes){
        
    }

    void dynamicDetector::dbscanDetect(std::vector<mapManager::box3D>& bboxes, std::vector<std::vector<Eigen::Vector3d>>& pcClusters){
        // 1. get pointcloud
        this->projectDepthImage();

        // 2. filter points
        this->filterPoints();

        // 3. cluster points and get bounding boxes
        this->clusterPointsAndBBoxes();

    }

    void dynamicDetector::projectDepthImage(){
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

                this->projPoints_[this->projPointsNum_] = currPointMap;
                this->projPointsNum_ = this->projPointsNum_ + 1;
            }
        } 
    }

    void dynamicDetector::filterPoints(){
        std::vector<Eigen::Vector3d> voxelFilteredPoints;
        this->voxelFilter(this->projPoints_, voxelFilteredPoints);
    }


    void dynamicDetector::clusterPointsAndBBoxes(){

    }

    void dynamicDetector::voxelFilter(const std::vector<Eigen::Vector3d>& points, std::vector<Eigen::Vector3d>& filteredPoints){
        const double res = 0.1; // resolution of voxel
        int xVoxels = ceil(this->localSensorRange_(0)/res); int yVoxels = ceil(this->localSensorRange_(1)/res); int zVoxels = ceil(this->localSensorRange_(2)/res);
        int totalVoxels = xVoxels * yVoxels * zVoxels;
        std::vector<bool> voxelOccupancyVec (totalVoxels, false);

        // Iterate through each points in the cloud
        filteredPoints.clear();
        for (size_t i=0; i<points.size(); ++i){
            Eigen::Vector3d p = points[i];

            if (this->isInFilterRange(p)){
                // find the corresponding voxel id in the vector and check whether it is occupied
                int pID = this->posToAddress(p, res);

                // if occupied, skip. Else add to the filtered points
                if (voxelOccupancyVec[pID] == true){
                    continue;
                }
                else{
                    voxelOccupancyVec[pID] = true;
                    filteredPoints.push_back(p);
                }
            }
        }  
    }
}

