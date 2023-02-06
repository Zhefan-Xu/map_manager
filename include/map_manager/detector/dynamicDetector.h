/*
    FILE: dynamicDetector.h
    ---------------------------------
    header file of dynamic obstacle detector
*/
#ifndef MAPMANAGER_DYNAMICDETECTOR_H
#define MAPMANAGER_DYNAMICDETECTOR_H

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <vision_msgs/Detection2DArray.h>
#include <image_transport/image_transport.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <map_manager/detector/dbscan.h>
#include <map_manager/detector/uv_detector.h>
#include <map_manager/detector/utils.h>

namespace mapManager{
    class dynamicDetector{
    private:
        std::string ns_;
        std::string hint_;

        // ROS
        ros::NodeHandle nh_;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depthSub_;
        std::shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> poseSub_;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, geometry_msgs::PoseStamped> depthPoseSync;
        std::shared_ptr<message_filters::Synchronizer<depthPoseSync>> depthPoseSync_;
        std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odomSub_;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry> depthOdomSync;
        std::shared_ptr<message_filters::Synchronizer<depthOdomSync>> depthOdomSync_;
        ros::Subscriber alignedDepthSub_; 
        ros::Subscriber yoloDetectionSub_;
        ros::Timer detectionTimer_;
        ros::Timer trackingTimer_;
        ros::Timer classificationTimer_;
        ros::Timer visTimer_;
        image_transport::Publisher uvDepthMapPub_;
        image_transport::Publisher uDepthMapPub_;
        image_transport::Publisher uvBirdViewPub_;
        image_transport::Publisher detectedAlignedDepthImgPub_;
        ros::Publisher uvBBoxesPub_;
        ros::Publisher filteredPointsPub_;
        ros::Publisher dbBBoxesPub_;
        ros::Publisher yoloBBoxesPub_;
        ros::Publisher dynamicBBoxesPub_;
        ros::Publisher filteredBoxesPub_;
        ros::Publisher historyTrajPub_;


        // DETECTOR
        std::shared_ptr<mapManager::UVdetector> uvDetector_;
        std::shared_ptr<mapManager::DBSCAN> dbCluster_;

        // CAMERA
        double fx_, fy_, cx_, cy_; // depth camera intrinsics
        double depthScale_; // value / depthScale
        double depthMinValue_, depthMaxValue_;
        int depthFilterMargin_, skipPixel_; // depth filter margin
        int imgCols_, imgRows_;
        Eigen::Matrix4d body2Cam_; // from body frame to camera frame

        // CAMERA ALIGNED DEPTH TO COLOR
        double fxC_, fyC_, cxC_, cyC_;
        Eigen::Matrix4d body2CamColor_;


        // DETECTOR PARAMETETER
        int localizationMode_;
        std::string depthTopicName_;
        std::string alignedDepthTopicName_;
        std::string poseTopicName_;
        std::string odomTopicName_;
        double raycastMaxLength_;
        double groundHeight_;
        int dbMinPointsCluster_;
        double dbEpsilon_;
        float boxIOUThresh_;
        double yoloThicknessRange_; 
        int histSize_;
        double dt_;
        double simThresh_;
        int skipFrame_;
        double dynaVelThresh_;
        double dynaVoteThresh_;
        double maxSkipRatio_;
        double voxelOccThresh_;

        // SENSOR DATA
        cv::Mat depthImage_;
        cv::Mat alignedDepthImage_;
        Eigen::Vector3d position_; // depth camera position
        Eigen::Matrix3d orientation_; // depth camera orientation
        Eigen::Vector3d positionColor_; // color camera position
        Eigen::Matrix3d orientationColor_; // color camera orientation
        Eigen::Vector3d localSensorRange_ {5.0, 5.0, 5.0};

        // DETECTOR DATA
        std::vector<mapManager::box3D> uvBBoxes_; // uv detector bounding boxes
        int projPointsNum_ = 0;
        std::vector<Eigen::Vector3d> projPoints_; // projected points from depth image
        std::vector<double> pointsDepth_;
        std::vector<Eigen::Vector3d> filteredPoints_; // filtered point cloud data
        std::vector<mapManager::box3D> dbBBoxes_; // DBSCAN bounding boxes        
        std::vector<std::vector<Eigen::Vector3d>> pcClusters_; // pointcloud clusters
        std::vector<mapManager::box3D> filteredBBoxes_; // filtered bboxes
        std::vector<std::vector<Eigen::Vector3d>> filteredPcClusters_; // pointcloud clusters after filtering by UV and DBSCAN fusion
        std::vector<mapManager::box3D> trackedBBoxes_; // bboxes tracked from kalman filtering
        std::vector<mapManager::box3D> dynamicBBoxes_; // boxes classified as dynamic


        // TRACKING AND ASSOCIATION DATA
        bool newDetectFlag_;
        std::vector<std::deque<mapManager::box3D>> boxHist_; // data association result: history of filtered bounding boxes for each box in current frame
        std::vector<std::deque<std::vector<Eigen::Vector3d>>> pcHist_; // data association result: history of filtered pc clusteres for each pc cluster in current frame
        std::deque<Eigen::Vector3d> positionHist_; // current position
		std::deque<Eigen::Matrix3d> orientationHist_; // current orientation
        


        std::vector<mapManager::box3D> yoloBBoxes_; // yolo detected bounding boxes
        vision_msgs::Detection2DArray yoloDetectionResults_; // yolo detected 2D results
        cv::Mat detectedAlignedDepthImg_;

    public:
        dynamicDetector();
        dynamicDetector(const ros::NodeHandle& nh);
        void initDetector(const ros::NodeHandle& nh);


        void initParam();
        void registerPub();
        void registerCallback();

        // callback
        void depthPoseCB(const sensor_msgs::ImageConstPtr& img, const geometry_msgs::PoseStampedConstPtr& pose);
        void depthOdomCB(const sensor_msgs::ImageConstPtr& img, const nav_msgs::OdometryConstPtr& odom);
        void alignedDepthCB(const sensor_msgs::ImageConstPtr& img);
        void yoloDetectionCB(const vision_msgs::Detection2DArrayConstPtr& detections);
        void detectionCB(const ros::TimerEvent&);
        void trackingCB(const ros::TimerEvent&);
        void classificationCB(const ros::TimerEvent&);
        void visCB(const ros::TimerEvent&);

        // detect function
        void uvDetect();
        void dbscanDetect();
        void filterBBoxes();
        void yoloDetectionTo3D();
        
        // uv Detector Functions
        void transformUVBBoxes(std::vector<mapManager::box3D>& bboxes);

        // DBSCAN Detector Functions
        void projectDepthImage();
        void filterPoints(const std::vector<Eigen::Vector3d>& points, std::vector<Eigen::Vector3d>& filteredPoints);
        void clusterPointsAndBBoxes(const std::vector<Eigen::Vector3d>& points, std::vector<mapManager::box3D>& bboxes, std::vector<std::vector<Eigen::Vector3d>>& pcClusters);
        void voxelFilter(const std::vector<Eigen::Vector3d>& points, std::vector<Eigen::Vector3d>& filteredPoints);
        void updatePoseHist();

        // detection helper functions
        float calBoxIOU(const mapManager::box3D& box1, const mapManager::box3D& box2);
        // float overlapLengthIfCIOU(float& overlap, float& l1, float& l2, mapManager::box3D& box1, mapManager::box3D& box2);// CIOU: complete-IOU

        // data association and tracking
        void boxAssociation();
        void boxAssociationHelper();
        void genFeat(const std::vector<mapManager::box3D>& propedBoxes, int numObjs, std::vector<Eigen::VectorXd>& propedBoxesFeat, std::vector<Eigen::VectorXd>& currBoxesFeat);
        void genFeatHelper(std::vector<Eigen::VectorXd>& feature, const std::vector<mapManager::box3D>& boxes);
        void linearProp(std::vector<mapManager::box3D>& propedBoxes);
        void findBestMatch(const std::vector<Eigen::VectorXd>& propedBoxesFeat, const std::vector<Eigen::VectorXd>& currBoxesFeat, const std::vector<mapManager::box3D>& propedBoxes, std::vector<int>& bestMatch);
        void updateHist(const std::vector<int>& bestMatch);
        

        // yolo helper functions
        void getYolo3DBBox(const vision_msgs::Detection2D& detection, mapManager::box3D& bbox3D, cv::Rect& bboxVis); 
        void calculateMAD(std::vector<double>& depthValues, double& depthMedian, double& MAD);


        // visualization
        void publishUVImages(); 
        void publishYoloImages();
        void publishPoints(const std::vector<Eigen::Vector3d>& points, const ros::Publisher& publisher);
        void publish3dBox(const std::vector<mapManager::box3D>& bboxes, const ros::Publisher& publisher, double r, double g, double b);
        void publishHistoryTraj();

        // helper function
        void transformBBox(const Eigen::Vector3d& center, const Eigen::Vector3d& size, const Eigen::Vector3d& position, const Eigen::Matrix3d& orientation,
                                  Eigen::Vector3d& newCenter, Eigen::Vector3d& newSize);
        bool isInFov(const Eigen::Vector3d& position, const Eigen::Matrix3d& orientation, Eigen::Vector3d& point);

        // inline helper functions
        bool isInFilterRange(const Eigen::Vector3d& pos);
        void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& idx, double res);
        int indexToAddress(const Eigen::Vector3i& idx, double res);
        int posToAddress(const Eigen::Vector3d& pos, double res);
        void indexToPos(const Eigen::Vector3i& idx, Eigen::Vector3d& pos, double res);
        void getCameraPose(const geometry_msgs::PoseStampedConstPtr& pose, Eigen::Matrix4d& camPoseMatrix, Eigen::Matrix4d& camPoseColorMatrix);
        void getCameraPose(const nav_msgs::OdometryConstPtr& odom, Eigen::Matrix4d& camPoseMatrix, Eigen::Matrix4d& camPoseColorMatrix);
        mapManager::Point eigenToDBPoint(const Eigen::Vector3d& p);
        Eigen::Vector3d dbPointToEigen(const mapManager::Point& pDB);
        void eigenToDBPointVec(const std::vector<Eigen::Vector3d>& points, std::vector<mapManager::Point>& pointsDB, int size);
    };


    inline bool dynamicDetector::isInFilterRange(const Eigen::Vector3d& pos){
        if ((pos(0) >= this->position_(0) - this->localSensorRange_(0)) and (pos(0) <= this->position_(0) + this->localSensorRange_(0)) and 
            (pos(1) >= this->position_(1) - this->localSensorRange_(1)) and (pos(1) <= this->position_(1) + this->localSensorRange_(1)) and 
            (pos(2) >= this->position_(2) - this->localSensorRange_(2)) and (pos(2) <= this->position_(2) + this->localSensorRange_(2))){
            return true;
        }
        else{
            return false;
        }        
    }

    inline void dynamicDetector::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& idx, double res){
        idx(0) = floor( (pos(0) - this->position_(0) + localSensorRange_(0)) / res);
        idx(1) = floor( (pos(1) - this->position_(1) + localSensorRange_(1)) / res);
        idx(2) = floor( (pos(2) - this->position_(2) + localSensorRange_(2)) / res);
    }

    inline int dynamicDetector::indexToAddress(const Eigen::Vector3i& idx, double res){
        // return idx(0) * ceil(2*this->localSensorRange_(1)/res) * ceil(2*this->localSensorRange_(2)/res) + idx(1) * ceil(2*this->localSensorRange_(2)/res) + idx(2);
        return idx(0) * ceil(this->localSensorRange_(0)/res) + idx(1) * ceil(this->localSensorRange_(1)/res) + idx(2);
    }

    inline int dynamicDetector::posToAddress(const Eigen::Vector3d& pos, double res){
        Eigen::Vector3i idx;
        this->posToIndex(pos, idx, res);
        return this->indexToAddress(idx, res);
    }

    inline void dynamicDetector::indexToPos(const Eigen::Vector3i& idx, Eigen::Vector3d& pos, double res){
		pos(0) = (idx(0) + 0.5) * res - localSensorRange_(0) + this->position_(0);
		pos(1) = (idx(1) + 0.5) * res - localSensorRange_(1) + this->position_(1);
		pos(2) = (idx(2) + 0.5) * res - localSensorRange_(2) + this->position_(2);
	}
    
    inline void dynamicDetector::getCameraPose(const geometry_msgs::PoseStampedConstPtr& pose, Eigen::Matrix4d& camPoseMatrix, Eigen::Matrix4d& camPoseColorMatrix){
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
        camPoseColorMatrix = map2body * this->body2CamColor_;
    }

    inline void dynamicDetector::getCameraPose(const nav_msgs::OdometryConstPtr& odom, Eigen::Matrix4d& camPoseMatrix, Eigen::Matrix4d& camPoseColorMatrix){
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
        camPoseColorMatrix = map2body * this->body2CamColor_;
    }
    
    inline mapManager::Point dynamicDetector::eigenToDBPoint(const Eigen::Vector3d& p){
        mapManager::Point pDB;
        pDB.x = p(0);
        pDB.y = p(1);
        pDB.z = p(2);
        pDB.clusterID = -1;
        return pDB;
    }

    inline Eigen::Vector3d dynamicDetector::dbPointToEigen(const mapManager::Point& pDB){
        Eigen::Vector3d p;
        p(0) = pDB.x;
        p(1) = pDB.y;
        p(2) = pDB.z;
        return p;
    }

    inline void dynamicDetector::eigenToDBPointVec(const std::vector<Eigen::Vector3d>& points, std::vector<mapManager::Point>& pointsDB, int size){
        for (int i=0; i<size; ++i){
            Eigen::Vector3d p = points[i];
            mapManager::Point pDB = this->eigenToDBPoint(p);
            pointsDB.push_back(pDB);
        }
    }

    

}

#endif