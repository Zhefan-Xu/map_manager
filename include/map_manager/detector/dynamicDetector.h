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
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <map_manager/detector/dbscan.h>
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
        ros::Timer detectionTimer_;
        ros::Timer trackingTimer_;
        ros::Timer classificationTimer_;

        // CAMERA
        double fx_, fy_, cx_, cy_; // depth camera intrinsics
        double depthScale_; // value / depthScale
        double depthMinValue_, depthMaxValue_;
        int depthFilterMargin_, skipPixel_; // depth filter margin
        int imgCols_, imgRows_;
        Eigen::Matrix4d body2Cam_; // from body frame to camera frame


        // DETECTOR PARAMETETER
        int localizationMode_;
        std::string depthTopicName_;
        std::string poseTopicName_;
        std::string odomTopicName_;
        double raycastMaxLength_;
        double groundHeight_;
        int dbMinPointsCluster_;
        double dbEpsilon_;



        // SENSOR DATA
        cv::Mat depthImage_;
        Eigen::Vector3d position_; // robot position
        Eigen::Matrix3d orientation_; // robot orientation
        Eigen::Vector3d localSensorRange_ {5.0, 5.0, 5.0};

        // DETECTOR DATA
        int projPointsNum_;
        std::vector<Eigen::Vector3d> projPoints_; // projected points from depth image
        std::vector<Eigen::Vector3d> filteredPoints_; // filtered point cloud data


    public:
        dynamicDetector();
        dynamicDetector(const ros::NodeHandle& nh);
        void initDetector(const ros::NodeHandle& nh);


        void initParam();
        void registerCallback();

        // callback
        void depthPoseCB(const sensor_msgs::ImageConstPtr& img, const geometry_msgs::PoseStampedConstPtr& pose);
        void depthOdomCB(const sensor_msgs::ImageConstPtr& img, const nav_msgs::OdometryConstPtr& odom);
        void detectionCB(const ros::TimerEvent&);
        void trackingCB(const ros::TimerEvent&);
        void classificationCB(const ros::TimerEvent&);

        // detect function
        void uvDetect(std::vector<mapManager::box3D>& bboxes);
        void dbscanDetect(std::vector<mapManager::box3D>& bboxes, std::vector<std::vector<Eigen::Vector3d>>& pcClusters);
    


        // DBSCAN Detector Functionns
        void projectDepthImage();
        void filterPoints();
        void clusterPointsAndBBoxes();
        void voxelFilter(const std::vector<Eigen::Vector3d>& points, std::vector<Eigen::Vector3d>& filteredPoints);

        // helper functions
        bool isInFilterRange(const Eigen::Vector3d& pos);
        void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& idx, double res);
        int indexToAddress(const Eigen::Vector3i& idx, double res);
        int posToAddress(const Eigen::Vector3d& pos, double res);
        void getCameraPose(const geometry_msgs::PoseStampedConstPtr& pose, Eigen::Matrix4d& camPoseMatrix);
        void getCameraPose(const nav_msgs::OdometryConstPtr& odom, Eigen::Matrix4d& camPoseMatrix);
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
        idx(0) = floor( (pos(0) - this->position_(0) - localSensorRange_(0)) / res);
        idx(1) = floor( (pos(1) - this->position_(1) - localSensorRange_(1)) / res);
        idx(2) = floor( (pos(2) - this->position_(2) - localSensorRange_(2)) / res);
    }

    inline int dynamicDetector::indexToAddress(const Eigen::Vector3i& idx, double res){
        return idx(0) * ceil(this->localSensorRange_(0)/res) + idx(1) * ceil(this->localSensorRange_(1)/res) + idx(2);
    }

    inline int dynamicDetector::posToAddress(const Eigen::Vector3d& pos, double res){
         Eigen::Vector3i idx;
         this->posToIndex(pos, idx, res);
         return this->indexToAddress(idx, res);
    }
    
    inline void dynamicDetector::getCameraPose(const geometry_msgs::PoseStampedConstPtr& pose, Eigen::Matrix4d& camPoseMatrix){
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

    inline void dynamicDetector::getCameraPose(const nav_msgs::OdometryConstPtr& odom, Eigen::Matrix4d& camPoseMatrix){
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

#endif