#include <map_manager/occupancyMap.h>
#include <map_manager/dynamicDetector.h>

#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <unsupported/Eigen/MatrixFunctions>

namespace mapManager{
    dynamicDetector::dynamicDetector(){
		this->ns_ = "dynamic_detector";
		this->hint_ = "[dynamicDetector]";
	}

	dynamicDetector::dynamicDetector(const ros::NodeHandle& nh, std::vector<Eigen::Vector3d>& projPoints, Eigen::Vector3d& position, Eigen::Vector3d& localMapSizeMin, Eigen::Vector3i& localMapVoxelMax, double& mapRes, double& depthMaxValue){
		this->ns_ = "dynamic_detector";
		this->hint_ = "[dynamicDetector]";
        this->nh_ = nh;
        this->projPoints_ = projPoints;
        
        this->position_ = position;
        this->localMapSizeMin_ = localMapSizeMin;
        this->localMapVoxelMax_ = localMapVoxelMax;
        this->mapRes_ = mapRes;
        this->depthMaxValue_ = depthMaxValue;
	}

    void dynamicDetector::initDetectorParam(){
        if (not this->nh_.getParam(this->ns_ + "/time_difference", this->dt_)){
			this->dt_ = 5;
			std::cout << this->hint_ << ": No time_difference parameter. Use default: 5." << std::endl;
		}
		else{
			std::cout << this->hint_ << ": time_difference is set to: " << this->dt_ << std::endl;
		}

        this->localPcVoxelSize_ = this->localMapVoxelMax_(0) * this->localMapVoxelMax_(1) * this->localMapVoxelMax_(2);
        this->localPcOccupied_.resize(this->localPcVoxelSize_, false);
    }

    void dynamicDetector::filteringAndClustering(){
        this->voxelFilter();

    }

    void dynamicDetector::voxelFilter(){
        Eigen::Vector3d currPoint;
        int localAddress;

        // filtering by voxel
        this->filteredPc_.clear();
        for (size_t i=0; i<this->projPoints_.size(); ++i){
			currPoint = this->projPoints_[i];
            localAddress = this->posToLocalAddress(currPoint);

            if (!this->localPcOccupied_[localAddress]){
                this->localPcOccupied_[localAddress] = true;
                if (this->pointsDepth_[i]<this->depthMaxValue_){
                    this->filteredPc_.push_back(currPoint);
                }
            }
        }

        //clean localPcVoxel
        this->localPcOccupied_.clear();
        this->localPcOccupied_.resize(this->localPcVoxelSize_, false);
    }
}

