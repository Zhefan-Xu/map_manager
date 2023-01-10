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

	dynamicDetector::dynamicDetector(const ros::NodeHandle& nh, Eigen::Vector3d& localMapSizeMin, Eigen::Vector3i& localMapVoxelMax, double& mapRes, double& depthMaxValue){
		this->ns_ = "dynamic_detector";
		this->hint_ = "[dynamicDetector]";
        this->nh_ = nh;
        this->localMapSizeMin_ = localMapSizeMin;
        this->localMapVoxelMax_ = localMapVoxelMax;
        this->mapRes_ = mapRes;
        this->depthMaxValue_ = depthMaxValue;

        this->initDetectorParam();
        this->dsCluster_.reset(new DBSCAN(this->minPoints_, this->epsilon_, this->dsPoints_));
	}

    void dynamicDetector::initDetectorParam(){
        if (not this->nh_.getParam(this->ns_ + "/time_difference", this->dt_)){
			this->dt_ = 5;
			std::cout << this->hint_ << ": No time_difference parameter. Use default: 5." << std::endl;
		}
		else{
			std::cout << this->hint_ << ": time_difference is set to: " << this->dt_ << std::endl;
		}

        if (not this->nh_.getParam(this->ns_ + "/ground_height", this->groundHeight_)){
			this->groundHeight_ = 0.1;
			std::cout << this->hint_ << ": No ground_height parameter. Use default: 0.1." << std::endl;
		}
		else{
			std::cout << this->hint_ << ": ground_height is set to: " << this->groundHeight_ << std::endl;
		}

        if (not this->nh_.getParam(this->ns_ + "/min_points", this->minPoints_)){
			this->minPoints_ = 18;
			std::cout << this->hint_ << ": No min_points parameter. Use default: 18." << std::endl;
		}
		else{
			std::cout << this->hint_ << ": min_points is set to: " << this->minPoints_ << std::endl;
		}

        if (not this->nh_.getParam(this->ns_ + "/epsilon", this->epsilon_)){
			this->epsilon_ = 0.3;
			std::cout << this->hint_ << ": No min_points parameter. Use default: 0.3." << std::endl;
		}
		else{
			std::cout << this->hint_ << ": epsilon is set to: " << this->epsilon_ << std::endl;
		}

        this->localPcVoxelSize_ = this->localMapVoxelMax_(0) * this->localMapVoxelMax_(1) * this->localMapVoxelMax_(2);
        this->localPcOccupied_.resize(this->localPcVoxelSize_, false);
    }

    void dynamicDetector::filteringAndClustering(){
        this->voxelFilter();
        this->neighborFilter();
        this->clustering();

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
                // filter out ground points and background points
                if (this->pointsDepth_[i]<this->depthMaxValue_ && currPoint(2)>this->groundHeight_){
                    this->filteredPc_.push_back(currPoint);
                }
            }
        }

        //clean localPcVoxel
        this->localPcOccupied_.clear();
        this->localPcOccupied_.resize(this->localPcVoxelSize_, false);
    }

    void dynamicDetector::neighborFilter(){

    }

    void dynamicDetector::clustering(){
        // set input
        this->dsCluster_->m_points.clear();
        Point currPoint;
        for (size_t i=0 ; i<this->filteredPc_.size() ; ++i){
            this->eigenToPointStruct(this->filteredPc_[i], currPoint);
            this->dsCluster_->m_points.push_back(currPoint);
        }

        // clustering
        this->dsCluster_->run();

        // test result
        // this->printClusterResults(this->dsCluster_->m_points, this->dsCluster_->m_points.size());

        // store cluster groups in this->clusters_
        this->dividePointsIntoClusters(); 
        this->clustersToBoxes();

    }

    void dynamicDetector::dividePointsIntoClusters(){
        this->clusters_.clear();

        // reize according to first point's index
        if (this->dsCluster_->m_points.size()){
            // set unclassifed as 0
            if (this->dsCluster_->m_points[0].clusterID == -1){
                this->dsCluster_->m_points[0].clusterID = 0;
            }
            this->clusters_.resize(this->dsCluster_->m_points[0].clusterID + 1); // reserve for unclassified points
        }

        for (size_t i=0 ; i<this->dsCluster_->m_points.size() ; ++i){
            
            Eigen::Vector3d currPc;
            Point currPoint = this->dsCluster_->m_points[i];

            // set unclassifed as 0
            if (currPoint.clusterID == -1){
                currPoint.clusterID = 0;
            }

            // point to eigen vector
            this->pointStructToEigen(currPc, currPoint);

            // insert points into cluster vectors
            if (currPoint.clusterID < int(this->clusters_.size())){ // insert to previous clusters
                this->clusters_[currPoint.clusterID].push_back(currPc);
            }
            else{ // push another cluster and insert pc
                std::vector<Eigen::Vector3d> currPcVec;
                currPcVec.push_back(currPc);
                this->clusters_.push_back(currPcVec);
            }
        }

        // show cluster and num of points
        // for (size_t i=0 ; i<this->clusters_.size() ; ++i){
        //     std::cout<<"cluster " << i << ": " << this->clusters_[i].size()<<std::endl;
        // }
    }

    void dynamicDetector::clustersToBoxes(){

        std::vector<box3D> boxesTemp;
        for (size_t i=1 ; i<this->clusters_.size() ; ++i){

            box3D box;

            float xmin = this->clusters_[i][0](0);
            float ymin = this->clusters_[i][0](1);
            float zmin = this->clusters_[i][0](2);
            float xmax = this->clusters_[i][0](0);
            float ymax = this->clusters_[i][0](1);
            float zmax = this->clusters_[i][0](2);

            for (size_t j=0 ; j<this->clusters_[i].size() ; ++j){
                xmin = (this->clusters_[i][j](0)<xmin)?this->clusters_[i][j](0):xmin;
                ymin = (this->clusters_[i][j](1)<ymin)?this->clusters_[i][j](1):ymin;
                zmin = (this->clusters_[i][j](2)<zmin)?this->clusters_[i][j](2):zmin;
                xmax = (this->clusters_[i][j](0)>xmax)?this->clusters_[i][j](0):xmax;
                ymax = (this->clusters_[i][j](1)>ymax)?this->clusters_[i][j](1):ymax;
                zmax = (this->clusters_[i][j](2)>zmax)?this->clusters_[i][j](2):zmax;
            }

            box.id = i;
            box.x = (xmax + xmin)/2.0;
            box.y = (ymax + ymin)/2.0;
            box.z = (zmax + zmin)/2.0;
            box.x_width = (xmax - xmin);
            box.y_width = (ymax - ymin);
            box.z_width = (zmax - zmin);
            boxesTemp.push_back(box);
        }

        this->obsBoxes_ = boxesTemp;
    }

    // tool functions
    void dynamicDetector::printClusterResults(std::vector<Point>& points, int num_points)
    {
        int i = 0;
        printf("Number of points: %u\n"
            " x     y     z     cluster_id\n"
            "-----------------------------\n"
            , num_points);
        while (i < num_points)
        {
            printf("%5.2lf %5.2lf %5.2lf: %d\n",
                    points[i].x,
                    points[i].y, points[i].z,
                    points[i].clusterID);
            ++i;
        }
    }

}

