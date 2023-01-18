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
    }

    void dynamicDetector::initDetector(const ros::NodeHandle& nh){
        this->nh_ = nh;
    }

}

