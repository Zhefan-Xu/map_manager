#ifndef MAPMANAGER_DYNAMICDETECTOR_H
#define MAPMANAGER_DYNAMICDETECTOR_H

#include <map_manager/occupancyMap.h>
#include <map_manager/dbscan.h>

#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <unsupported/Eigen/MatrixFunctions>


namespace mapManager{

    struct box3D
    {
        /* data */
        float x, y, z;
        float x_width, y_width, z_width;
        float id;
    };
    

    class dynamicDetector : public occMap{
    private:
        /* data */

        //PARAMS
        int dt_;
        float groundHeight_;
        int minPoints_;
        float epsilon_;
        
        // filteringAndClustering
        int localPcVoxelSize_;
        std::vector<bool> localPcOccupied_; //voxel for projected point in current frame
        std::vector<Eigen::Vector3d> filteredPc_; // filtered piontcloud in current frame
        std::shared_ptr<DBSCAN> dsCluster_;
        std::vector<Point> dsPoints_;
        std::vector<std::vector<Eigen::Vector3d>> clusters_; // results of clustering
        std::vector<box3D> obsBoxes_;

        // ROS
        // ros::NodeHandle nh;
        
    public:
        dynamicDetector(/* args */);
        dynamicDetector(const ros::NodeHandle& nh, Eigen::Vector3d& localMapSizeMin, Eigen::Vector3i& localMapVoxelMax, double& mapRes, double& depthMaxValue);

        
        void initDetectorParam();
        void filteringAndClustering();
		void occlusionAwareTracking();
		void identifyDynamicObs();
        
        // filtering and clustering
        void voxelFilter();
        void neighborFilter();
        void clustering();
        void dividePointsIntoClusters();
        void clustersToBoxes();

        // user interface
        void getFilteredPc(std::vector<Eigen::Vector3d>& incomePc);
        void getObsBoxes(std::vector<box3D>& incomeBoxes);
        void setPointsDepth(std::vector<double>& incomePointsDepth);
        void setProjPoints(std::vector<Eigen::Vector3d>& incomeProjPoints);
        void setPosition(Eigen::Vector3d& incomePosition);

        // tool functions
        void posToLocalIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& localIdx);
		int localIndexToLocalAddress(const Eigen::Vector3i& localIdx);
        int posToLocalAddress(const Eigen::Vector3d& pos);
        void eigenToPointStruct(const Eigen::Vector3d& eigenVec, Point& point);
        void pointStructToEigen(Eigen::Vector3d& eigenVec, const Point& point);
        void printClusterResults(std::vector<Point>& points, int num_points);


    };
    
    // tool functions
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

    inline void dynamicDetector::eigenToPointStruct(const Eigen::Vector3d& eigenVec, Point& point){
        point.x = eigenVec(0);
        point.y = eigenVec(1);
        point.z = eigenVec(2);
        point.clusterID = -1; // UNCLASIFIED = -1
    }

    inline void dynamicDetector::pointStructToEigen(Eigen::Vector3d& eigenVec, const Point& point){
        eigenVec(0) = point.x;
        eigenVec(1) = point.y;
        eigenVec(2) = point.z;
    }
    
    // usr interface
    inline void dynamicDetector::getFilteredPc(std::vector<Eigen::Vector3d>& incomePc){
        incomePc = this->filteredPc_;
    }

    inline void dynamicDetector::getObsBoxes(std::vector<box3D>& incomeBoxes){
        incomeBoxes = this->obsBoxes_;
    }

    inline void dynamicDetector::setPointsDepth(std::vector<double>& incomePointsDepth){
        this->pointsDepth_ = incomePointsDepth;
    }

    inline void dynamicDetector::setProjPoints(std::vector<Eigen::Vector3d>& incomeProjPoints){
        this->projPoints_ = incomeProjPoints;
    }

    inline void dynamicDetector::setPosition(Eigen::Vector3d& incomePosition){
        this->position_ = incomePosition;
    }
    
}

#endif