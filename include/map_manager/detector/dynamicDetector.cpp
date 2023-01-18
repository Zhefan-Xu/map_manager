/*
    FILE: dynamicDetector.cpp
    ---------------------------------
    function implementation of dynamic osbtacle detector
*/
#include <map_manager/detector/dynamicDetector.h>


namespace mapManager{
    dynamicDetector::dynamicDetector(){}

    dynamicDetector::dynamicDetector(const ros::NodeHandle& nh){
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
        // ground height
        if (not this->nh_.getParam("dynamic_detector/ground_height", this->groundHeight_)){
            this->groundHeight_ = 0.1;
            std::cout << "[dynamicDetector]: No ground height parameter. Use default: 0.1m." << std::endl;
        }
        else{
            std::cout << "[dynamicDetector]: Ground height is set to: " << this->groundHeight_ << std::endl;
        }

        // minimum number of points in each cluster
        if (not this->nh_.getParam("dynamic_detector/dbscan_min_points_cluster", this->dbMinPointsCluster_)){
            this->dbMinPointsCluster_ = 18;
            std::cout << "[dynamicDetector]: No DBSCAN minimum point in each cluster parameter. Use default: 18." << std::endl;
        }
        else{
            std::cout << "[dynamicDetector]: DBSCAN Minimum point in each cluster is set to: " << this->dbMinPointsCluster_ << std::endl;
        }

        // search range
        if (not this->nh_.getParam("dynamic_detector/dbscan_search_range_epsilon", this->dbEpsilon_)){
            this->dbEpsilon_ = 0.3;
            std::cout << "[dynamicDetector]: No DBSCAN epsilon parameter. Use default: 0.3." << std::endl;
        }
        else{
            std::cout << "[dynamicDetector]: DBSCAN epsilon is set to: " << this->dbEpsilon_ << std::endl;
        }  
    }

    void dynamicDetector::registerCallback(){
        // detection timer
        this->detectionTimer_ = this->nh_.createTimer(ros::Duration(0.033), &dynamicDetector::detectionCB, this);

        // tracking timer
        this->trackingTimer_ = this->nh_.createTimer(ros::Duration(0.033), &dynamicDetector::trackingCB, this);

        // classification timer
        this->classificationTimer_ = this->nh_.createTimer(ros::Duration(0.033), &dynamicDetector::classificationCB, this);
    }


    void dynamicDetector::detectionCB(const ros::TimerEvent&){
        cout << "detector CB" << endl;
    }

    void dynamicDetector::trackingCB(const ros::TimerEvent&){
        cout << "tracking CB" << endl;
        // data association


        // kalman filter tracking

    }

    void dynamicDetector::classificationCB(const ros::TimerEvent&){
        cout << "classification CB" << endl;
    }

    void dynamicDetector::uvDetect(){
        
    }

    void dynamicDetector::dbscanDetect(){

    }


}

