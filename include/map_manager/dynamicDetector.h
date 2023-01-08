#ifndef MAPMANAGER_DYNAMICDETECTOR_H
#define MAPMANAGER_DYNAMICDETECTOR_H

#include <map_manager/occupancyMap.h>

#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <unsupported/Eigen/MatrixFunctions>


namespace mapManager{
    class dynamicDetector : public occMap{
    private:
        /* data */

        //PARAMS
        int dt_;
        
        // filteringAndClustering
        int localPcVoxelSize_;
        std::vector<bool> localPcOccupied_; //voxel for projected point in current frame
        std::vector<Eigen::Vector3d> filteredPc_; // filtered piontcloud in current frame

        // ROS
        ros::NodeHandle nh;
        
    public:
        dynamicDetector(/* args */);
        dynamicDetector(const ros::NodeHandle& nh, std::vector<Eigen::Vector3d>& projPoints, Eigen::Vector3d& position, Eigen::Vector3d& localMapSizeMin, Eigen::Vector3i& localMapVoxelMax, double& mapRes);

        
        void initDetectorParam();
        void filteringAndClustering();
		void occlusionAwareTracking();
		void identifyDynamicObs();
        
        // filtering and clustering
        void voxelFilter();

        // user interface
        void getFilteredPc(std::vector<Eigen::Vector3d>& incomePc);
        void setProjPoints(std::vector<Eigen::Vector3d>& incomeProjPoints);
        void setPosition(Eigen::Vector3d& incomePosition);

        // tool functions
        void posToLocalIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& localIdx);
		int localIndexToLocalAddress(const Eigen::Vector3i& localIdx);
        int posToLocalAddress(const Eigen::Vector3d& pos);


    };

    inline void dynamicDetector::getFilteredPc(std::vector<Eigen::Vector3d>& incomePc){
        incomePc = this->filteredPc_;
    }
    
    inline void dynamicDetector::posToLocalIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& localIdx){
        localIdx(0) = floor( (pos(0) - this->position_(0) - this->localMapSizeMin_(0) ) / this->mapRes_ );
		localIdx(1) = floor( (pos(1) - this->position_(1) - this->localMapSizeMin_(1) ) / this->mapRes_ );
		localIdx(2) = floor( (pos(2) - this->position_(2) - this->localMapSizeMin_(2) ) / this->mapRes_ );   
    }

	inline int dynamicDetector::localIndexToLocalAddress(const Eigen::Vector3i& localIdx){
		return localIdx(0) * this->localMapVoxelMax_(1) * this->localMapVoxelMax_(2) + localIdx(1) * this->localMapVoxelMax_(2) + localIdx(2);
	}
    
    inline int dynamicDetector::posToLocalAddress(const Eigen::Vector3d& pos){
		Eigen::Vector3i localIdx;
		this->posToLocalIndex(pos, localIdx);
		return this->localIndexToLocalAddress(localIdx);
	}
    
    inline void dynamicDetector::setProjPoints(std::vector<Eigen::Vector3d>& incomeProjPoints){
        this->projPoints_ = incomeProjPoints;
    }
    inline void dynamicDetector::setPosition(Eigen::Vector3d& incomePosition){
        this->position_ = incomePosition;
    }
    
}

#endif