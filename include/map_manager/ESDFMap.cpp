/*
	FILE: ESDFMap.h
	--------------------------------------
	definition of ESDF map
*/
#include <map_manager/ESDFMap.h>

namespace mapManager{
	ESDFMap::ESDFMap(){
		this->ns_ = "esdf_map";
		this->hint_ = "[ESDFMap]";
	}

	void ESDFMap::initMap(const ros::NodeHandle& nh){
		this->nh_ = nh;
		this->initParam();
		this->initESDFParam();
		this->registerPub();
		this->registerESDFPub();
		this->registerCallback();
		this->registerESDFCallback();
	}

	void ESDFMap::initESDFParam(){
		int reservedSize = this->occupancy_.size();
		this->esdfTemp1_.resize(reservedSize, 0);
		this->esdfTemp2_.resize(reservedSize, 0);
		this->esdfDistancePos_.resize(reservedSize, 10000);
		this->esdfDistanceNeg_.resize(reservedSize, 10000);
		this->esdfDistance_.resize(reservedSize, 10000);
	}

	void ESDFMap::registerESDFPub(){
		this->esdfPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>(this->ns_ + "/esdf", 10);
	}

	void ESDFMap::registerESDFCallback(){
		this->esdfTimer_ = this->nh_.createTimer(ros::Duration(0.05), &ESDFMap::updateESDFCB, this);
		this->esdfPubTimer_ = this->nh_.createTimer(ros::Duration(0.05), &ESDFMap::ESDFPubCB, this);
	}

	void ESDFMap::updateESDFCB(const ros::TimerEvent& ){
		ros::Time startTime, endTime;
		startTime = ros::Time::now();
		if (this->esdfNeedUpdate_){
			this->updateESDF3D();
			this->esdfNeedUpdate_ = false;
		}
		endTime = ros::Time::now();
		cout << this->hint_ << ": ESDF update time: " << (endTime - startTime).toSec() << " s." << endl; 
	}

	void ESDFMap::updateESDF3D(){
		Eigen::Vector3i minRange = this->localBoundMin_;
		Eigen::Vector3i maxRange = this->localBoundMax_;

		// positive DT
		for (int x=minRange(0); x<=maxRange(0); ++x){
			for (int y=minRange(1); y<=maxRange(1); ++y){
		  		this->fillESDF([&](int z){return this->occupancyInflated_[this->indexToAddress(x, y, z)] == 1 ? 0 : std::numeric_limits<double>::max();},
		      			 [&](int z, double val) {this->esdfTemp1_[this->indexToAddress(x, y, z)] = val;}, minRange(2), maxRange(2), 2);
			}
		}

		for (int x=minRange(0); x<=maxRange(0); ++x){
			for (int z=minRange(2); z<=maxRange(2); ++z){
				this->fillESDF([&](int y){return this->esdfTemp1_[this->indexToAddress(x, y, z)];},
					     [&](int y, double val){this->esdfTemp2_[this->indexToAddress(x, y, z)]=val;}, minRange(1), maxRange(1), 1);
			}
		}

		for (int y=minRange(1); y<=maxRange(1); ++y){
			for (int z=minRange(2); z<=maxRange(2); ++z){
		  		this->fillESDF([&](int x) {return this->esdfTemp2_[this->indexToAddress(x, y, z)];},
		           		 	   [&](int x, double val){this->esdfDistancePos_[indexToAddress(x, y, z)]=this->mapRes_ * std::sqrt(val);}, minRange(0), maxRange(0), 0);
			}
		}

		// negative DT
		for (int x=minRange(0); x<=maxRange(0); ++x){
			for (int y=minRange(1); y<=maxRange(1); ++y){
				this->fillESDF([&](int z){return this->occupancyInflated_[this->indexToAddress(x, y, z)] == 0? 0 : std::numeric_limits<double>::max();},
		  		[&](int z, double val){this->esdfTemp1_[this->indexToAddress(x, y, z)]=val;}, minRange(2), maxRange(2), 2);
			}
		}

		for (int x=minRange(0); x<=maxRange(0); ++x){
			for (int z=minRange(2); z<=maxRange(2); ++z){
				this->fillESDF([&](int y) {return this->esdfTemp1_[this->indexToAddress(x, y, z)];},
		       				   [&](int y, double val){this->esdfTemp2_[this->indexToAddress(x, y, z)]=val;}, minRange(1), maxRange(1), 1);
			}
		}

		for (int y=minRange(1); y<=maxRange(1); ++y){
			for (int z=minRange(2); z<=maxRange(2); ++z){
				this->fillESDF([&](int x){return this->esdfTemp2_[this->indexToAddress(x, y, z)];},
		       				   [&](int x, double val){this->esdfDistanceNeg_[this->indexToAddress(x, y, z)]=this->mapRes_ * std::sqrt(val);}, minRange(0), maxRange(0), 0);
			}
		}

		// combine positive and negative DT
		for (int x=minRange(0); x<=maxRange(0); ++x){
			for (int y=minRange(1); y<=maxRange(1); ++y){
				for (int z=minRange(2); z<=maxRange(2); ++z){
					int idx = this->indexToAddress(x, y, z);
					this->esdfDistance_[idx] = this->esdfDistancePos_[idx];

					if (this->esdfDistanceNeg_[idx]>0.0){
				  		this->esdfDistance_[idx] += (-this->esdfDistanceNeg_[idx] + this->mapRes_);
					}
				}
			}
		}
	}

	template <typename F_get_val, typename F_set_val>
	void ESDFMap::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim){
		int v[this->mapVoxelMax_(dim)];
		double z[this->mapVoxelMax_(dim) + 1];

		int k = start;
		v[start] = start;
		z[start] = -std::numeric_limits<double>::max();
		z[start + 1] = std::numeric_limits<double>::max();

		for (int q = start + 1; q <= end; q++) {
			k++;
			double s;

			do {
				k--;
				s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
			}while (s <= z[k]);

			k++;

			v[k] = q;
			z[k] = s;
			z[k + 1] = std::numeric_limits<double>::max();
		}

		k = start;

		for (int q = start; q <= end; q++) {
			while (z[k + 1] < q) k++;
			double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
			f_set_val(q, val);
		}		
	}

	double ESDFMap::getDistance(const Eigen::Vector3d& pos){
		Eigen::Vector3i idx;
		this->posToIndex(pos, idx);
		return this->getDistance(idx);
	}

	double ESDFMap::getDistance(const Eigen::Vector3i& idx){
		Eigen::Vector3i idx1 = idx;
		this->boundIndex(idx1);
		return this->esdfDistance_[this->indexToAddress(idx1)];
	}

	void ESDFMap::ESDFPubCB(const ros::TimerEvent& ){
		this->publishESDF();
	}

	void ESDFMap::publishESDF(){
		double dist;
		pcl::PointCloud<pcl::PointXYZI> cloud;
		pcl::PointXYZI pt;

		const double minDist = 0.0;
		const double maxDist = 3.0;

		Eigen::Vector3i minRange = this->localBoundMin_;
		Eigen::Vector3i maxRange = this->localBoundMax_;
		this->boundIndex(minRange);
		this->boundIndex(maxRange);

		Eigen::Vector3d pos;
		double height = this->position_(2);
		for (int x=minRange(0); x<=maxRange(0); ++x){
			for (int y=minRange(1); y<=maxRange(1); ++y){
				this->indexToPos(Eigen::Vector3i(x, y, 1), pos);
				pos(2) = height;

				dist = this->getDistance(pos);
				dist = std::min(dist, maxDist);
				dist = std::max(dist, minDist);

				pt.x = pos(0);
				pt.y = pos(1);
				pt.z = height;
				pt.intensity = (dist - minDist)/(maxDist - minDist);
				cloud.push_back(pt);
			}
		}

		cloud.width = cloud.points.size();
		cloud.height = 1;
		cloud.is_dense = true;
		cloud.header.frame_id = "map";
		sensor_msgs::PointCloud2 cloud_msg;
		pcl::toROSMsg(cloud, cloud_msg);

		this->esdfPub_.publish(cloud_msg);		
	}


	// inline functions
	inline void occMap::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& idx){
		idx(0) = floor( (pos(0) - this->mapSizeMin_(0) ) / this->mapRes_ );
		idx(1) = floor( (pos(1) - this->mapSizeMin_(1) ) / this->mapRes_ );
		idx(2) = floor( (pos(2) - this->mapSizeMin_(2) ) / this->mapRes_ );
	}

	inline void occMap::indexToPos(const Eigen::Vector3i& idx, Eigen::Vector3d& pos){
		pos(0) = (idx(0) + 0.5) * this->mapRes_ + this->mapSizeMin_(0); 
		pos(1) = (idx(1) + 0.5) * this->mapRes_ + this->mapSizeMin_(1);
		pos(2) = (idx(2) + 0.5) * this->mapRes_ + this->mapSizeMin_(2);
	}

	inline int occMap::posToAddress(const Eigen::Vector3d& pos){
		Eigen::Vector3i idx;
		this->posToIndex(pos, idx);
		return this->indexToAddress(idx);
	}

	inline int occMap::posToAddress(double x, double y, double z){
		Eigen::Vector3d pos (x, y, z);
		return this->posToAddress(pos);
	}

	inline int occMap::indexToAddress(const Eigen::Vector3i& idx){
		return idx(0) * this->mapVoxelMax_(1) * this->mapVoxelMax_(2) + idx(1) * this->mapVoxelMax_(2) + idx(2);
	}

	inline int occMap::indexToAddress(int x, int y, int z){
		Eigen::Vector3i idx (x, y, z);
		return this->indexToAddress(idx);
	}

	inline void occMap::boundIndex(Eigen::Vector3i& idx){
		Eigen::Vector3i temp;
		temp(0) = std::max(std::min(idx(0), this->mapVoxelMax_(0)), this->mapVoxelMin_(0));
		temp(1) = std::max(std::min(idx(1), this->mapVoxelMax_(1)), this->mapVoxelMin_(1));
		temp(2) = std::max(std::min(idx(2), this->mapVoxelMax_(2)), this->mapVoxelMin_(2));
		idx = temp;
	}
}