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

	dynamicDetector::dynamicDetector(const ros::NodeHandle& nh, std::vector<Eigen::Vector3d> projPoints){
		this->ns_ = "dynamic_detector";
		this->hint_ = "[dynamicDetector]";
        this->nh_ = nh;
        this->projPoints_ = projPoints;
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
        Eigen::Vector3d currPoint;
        int localAddress;
        int count;
        ROS_INFO("here1");
        std::cout<<"projPointsNum, projPoints size: %i, %i"<<this->projPointsNum_ << " " << this->projPoints_.size()<<std::endl;
        // voxel filter
        for (size_t i=0; i<this->projPoints_.size(); ++i){
			currPoint = this->projPoints_[i];
            ROS_INFO("herer2");
            localAddress = this->posToLocalAddress(currPoint);
            if (!this->localPcOccupied_[localAddress]){
                // ROS_INFO("local address: %i", localAddress);
                std::cout<<"localAddress"<<localAddress<<std::endl;
                this->localPcOccupied_[localAddress] = true;
                this->filteredPc_.push_back(currPoint);
                count++;
            }
        }
        
        // std::cout<<"filtered pc size: "<<this->filteredPc_.size()<<std::endl;
        // std::cout << "count "<< count << std::endl;
        //clean localPcVoxel
        this->localPcOccupied_.clear();
        this->localPcOccupied_.resize(this->localPcVoxelSize_, false);

    }
}

