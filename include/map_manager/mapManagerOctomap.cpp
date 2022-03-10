/*
	FILE: mapManagerOctomap.cpp
	---------------------------
	Function definition of octomap manager
*/
#include<map_manager/mapManagerOctomap.h>

namespace mapManager{
	mapManagerOctomap::mapManagerOctomap(const ros::NodeHandle& nh) : nh_(nh){
		if (not this->nh_.getParam("map_resolution", this->mapRes_)){
			this->mapRes_ = 0.2;
			cout << "[Map Manager]: No Map Resolition Parameter. Use default map resolution: 0.2." << endl;
		}

		this->mapClient_ = this->nh_.serviceClient<octomap_msgs::GetOctomap>("/octomap_binary");
		this->updateMap();

		// start collision checking server:
		this->collisionServer_ = this->nh_.advertiseService("check_collision_octomap", &mapManagerOctomap::checkCollision, this);
		// start random sample server:
		this->sampleServer_ = this->nh_.advertiseService("randome_sample_octomap", &mapManagerOctomap::randomSample, this);

		ros::spin();
	}

	void mapManagerOctomap::updateMap(){
		octomap_msgs::GetOctomap mapSrv;
		bool serviceSuccess = this->mapClient_.call(mapSrv);
		ros::Rate rate(10);
		while (not serviceSuccess and ros::ok()){
			serviceSuccess = this->mapClient_.call(mapSrv);
			ROS_INFO("[Map Manager Octomap]: Wait for octomap service...");
			rate.sleep();
		}
		octomap::AbstractOcTree* abtree = octomap_msgs::binaryMsgToMap(mapSrv.response.map);
		this->map_ = dynamic_cast<octomap::OcTree*>(abtree);
		this->updateSampleRegion();
		cout << "[Map Manager Octomap]: Map updated!" << endl;
	}

	void mapManagerOctomap::updateSampleRegion(){
		double xmin, xmax, ymin, ymax, zmin, zmax;
		this->map_->getMetricMax(xmax, ymax, zmax);
		this->map_->getMetricMin(xmin, ymin, zmin);
		this->sampleRegion_[0] = xmin;
		this->sampleRegion_[1] = xmax;
		this->sampleRegion_[2] = ymin;
		this->sampleRegion_[3] = ymax;
		this->sampleRegion_[4] = zmin;
		this->sampleRegion_[5] = zmax;
	}

	bool mapManagerOctomap::checkCollisionPoint(const octomap::point3d &p, bool ignoreUnknown){
		octomap::OcTreeNode* nptr = this->map_->search(p);
		if (nptr == NULL){
			if (not ignoreUnknown){
				return true;
			}
			else{
				return false;
			}
		}
		return this->map_->isNodeOccupied(nptr);
	}

	bool mapManagerOctomap::checkCollision(const octomap::point3d &p, double xsize, double ysize, double zsize, bool ignoreUnknown){
		double xmin, xmax, ymin, ymax, zmin, zmax; // bounding box for collision checking
		xmin = p.x() - xsize/2; xmax = p.x() + xsize/2;
		ymin = p.y() - ysize/2; ymax = p.y() + ysize/2;
		zmin = p.z() - zsize/2; zmax = p.z() + zsize/2;

		int xNum = (xmax - xmin)/this->mapRes_;
		int yNum = (ymax - ymin)/this->mapRes_;
		int zNum = (zmax - zmin)/this->mapRes_;

		int xID, yID, zID;
		for (xID=0; xID<=xNum; ++xID){
			for (yID=0; yID<=yNum; ++yID){
				for (zID=0; zID<=zNum; ++zID){
					if (this->checkCollisionPoint(octomap::point3d(xmin+xID*this->mapRes_, ymin+yID*this->mapRes_, zmin+zID*this->mapRes_), false)){
						return true;
					}
				}
			}
		}
		return false;
	}

	bool mapManagerOctomap::checkCollision(map_manager::CheckCollision::Request &req, map_manager::CheckCollision::Response &res){
		double px = req.x;
		double py = req.y;
		double pz = req.z;
		double xsize = req.xsize;
		double ysize = req.ysize;
		double zsize = req.zsize;
		bool updateMap = req.update_map;
		bool ignoreUnknown = req.ignore_unknown;
		if (updateMap){
			this->updateMap();
		}
		bool hasCollision = this->checkCollision(octomap::point3d(px, py, pz), xsize, ysize, zsize, ignoreUnknown);
		res.hasCollision = hasCollision;
		return true;
	}


	octomap::point3d mapManagerOctomap::randomSample(double xsize, double ysize, double zsize, bool ignoreUnknown){
		bool valid = false;
		double x, y, z;
		static octomap::point3d p;
		while (not valid){
			p.x() = randomNumber(this->sampleRegion_[0], this->sampleRegion_[1]);
			p.y() = randomNumber(this->sampleRegion_[2], this->sampleRegion_[3]);
			p.z() = randomNumber(this->sampleRegion_[4], this->sampleRegion_[5]);
			valid = not this->checkCollision(p, xsize, ysize, zsize, ignoreUnknown);
		}
		return p;
	}

	bool mapManagerOctomap::randomSample(map_manager::RandomSample::Request &req, map_manager::RandomSample::Response &res){
		double xsize = req.xsize;
		double ysize = req.ysize;
		double zsize = req.zsize;
		bool ignoreUnknown = req.ignore_unknown;
		octomap::point3d p = this->randomSample(xsize, ysize, zsize, ignoreUnknown);
		res.x = p.x();
		res.y = p.y();
		res.z = p.z();
		res.yaw = randomNumber(-PI_const, PI_const);
		return true;
	}
}