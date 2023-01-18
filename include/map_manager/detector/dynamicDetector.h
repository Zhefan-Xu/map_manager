/*
    FILE: dynamicDetector.h
    ---------------------------------
    header file of dynamic obstacle detector
*/
#ifndef MAPMANAGER_DYNAMICDETECTOR_H
#define MAPMANAGER_DYNAMICDETECTOR_H

#include <ros/ros.h>
#include <map_manager/detector/dbscan.h>


namespace mapManager{
    class dynamicDetector{
    private:
        ros::NodeHandle nh_;
        ros::Timer detectionTimer_;
        ros::Timer trackingTimer_;
        ros::Timer classificationTimer_;
        
        // detector parameters
        double groundHeight_;
        int dbMinPointsCluster_;
        double dbEpsilon_;

    public:
        dynamicDetector();
        dynamicDetector(const ros::NodeHandle& nh);
        void initDetector(const ros::NodeHandle& nh);
        

        void initParam();
        void registerCallback();

        // callback
        void detectionCB(const ros::TimerEvent&);
        void trackingCB(const ros::TimerEvent&);
        void classificationCB(const ros::TimerEvent&);

        // detect function
        void uvDetect();
        void dbscanDetect();
    };
    

    
}

#endif