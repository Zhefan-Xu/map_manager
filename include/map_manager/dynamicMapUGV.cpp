/*
	FILE: dynamicMapUGV.cpp
	--------------------------------------
	function definition of dynamic map with ugv info combined
*/

#include <ros/ros.h>
#include <map_manager/dynamicMapUGV.h>



namespace mapManager{

	dynamicMapUGV::dynamicMapUGV(){
		this->ns_ = "dynamic_map_ugv";
		this->hint_ = "[dynamicMapUGV]";
	}

	dynamicMapUGV::dynamicMapUGV(const ros::NodeHandle& nh){
		this->ns_ = "dynamic_map_ugv";
		this->hint_ = "[dynamicMapUGV]";
		this->initMap(nh);
	}

	void dynamicMapUGV::initMap(const ros::NodeHandle& nh){
		this->nh_ = nh;
		this->initParam();
		this->initDynamicUGVParam();
		this->initPreloadMap();
		this->registerPub();
		this->registerCallback();
		this->detector_.reset(new mapManager::dynamicDetector (this->nh_));
        this->freeMapTimer_ = this->nh_.createTimer(ros::Duration(0.01), &dynamicMapUGV::freeMapCB, this);
        this->visTimer_ = this->nh_.createTimer(ros::Duration(0.033), &dynamicMapUGV::visCB, this);
		this->lidarSub_ = this->nh_.subscribe(this->lidarDetectTopicName_,10, &dynamicMapUGV::lidarBoxCB,this);

	}

	void dynamicMapUGV::initDynamicUGVParam(){
		// lidar detected bounding box topic name
		if (not this->nh_.getParam(this->ns_ + "/lidar_detect_topic", this->lidarDetectTopicName_)){
			this->lidarDetectTopicName_ = "/lidar_detector/Detection_bounding_box";
			cout << this->hint_ << ": No lidar detection bounding box topic name. Use default: depth image" << endl;
		}
		else{
			cout << this->hint_ << ": The topic name of lidar detection bounding box is: " << this->lidarDetectTopicName_ << endl;
		}	
	}

	void dynamicMapUGV::registerPub(){
		this->depthCloudPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>(this->ns_ + "/depth_cloud", 10);
		this->mapVisPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>(this->ns_ + "/voxel_map", 10);
		this->inflatedMapVisPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>(this->ns_ + "/inflated_voxel_map", 10);
		this->map2DPub_ = this->nh_.advertise<nav_msgs::OccupancyGrid>(this->ns_ + "/2D_occupancy_map", 10);
		this->mergedLidarBoxesPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->ns_ + "/merged_lidar_boxes", 10);
		// this->visWorker_ = std::thread(&occMap::startVisualization, this);
	}

	void dynamicMapUGV::visCB(const ros::TimerEvent& ){
		this->publishProjPoints();
		this->publishMap();
		this->publishInflatedMap();
		this->publish2DOccupancyGrid();
		this->publish3dBox(this->lidarBoxes_, this->mergedLidarBoxesPub_, 0.3,0.6,0.3);
	}

	void dynamicMapUGV::freeMapCB(const ros::TimerEvent&){
		std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> freeRegions;
		std::vector<mapManager::box3D> dynamicBBoxes;
		this->detector_->getDynamicObstacles(dynamicBBoxes);
		for (mapManager::box3D ob:dynamicBBoxes){
			Eigen::Vector3d lowerBound (ob.x-ob.x_width/2-2*this->mapRes_-this->robotSize_(0)/2, ob.y-ob.y_width/2-2*this->mapRes_-this->robotSize_(1)/2, 0.0);
			Eigen::Vector3d upperBound (ob.x+ob.x_width/2+2*this->mapRes_+this->robotSize_(0)/2, ob.y+ob.y_width/2+2*this->mapRes_+this->robotSize_(1)/2, ob.z+ob.z_width+2*this->mapRes_+this->robotSize_(2)/2);
			freeRegions.push_back(std::make_pair(lowerBound, upperBound));
		}
		this->updateFreeRegions(freeRegions);
	}

    void dynamicMapUGV::lidarBoxCB(const vision_msgs::Detection3DArrayConstPtr& box_msg) 
    {
		cout <<"get lidar box" << endl;
		double dt = 0.1;
		int histSize = 5;
        std::vector<box3D> lidarBoxesTemp;
		lidarBoxesTemp.resize(box_msg->detections.size());
        for(int i=0; i<box_msg->detections.size();i++)
        {    
            std::cout << "Position X " <<box_msg->detections[i].bbox.center.position.x << std::endl;
            std::cout << "Position Y " <<box_msg->detections[i].bbox.center.position.y << std::endl;
            std::cout << "Position Z " <<box_msg->detections[i].bbox.center.position.z << std::endl;
            lidarBoxesTemp[i].x = box_msg->detections[i].bbox.center.position.x;
            lidarBoxesTemp[i].y = box_msg->detections[i].bbox.center.position.y;
            lidarBoxesTemp[i].z = box_msg->detections[i].bbox.center.position.z;
            
        }
        this->lidarBoxes_ = lidarBoxesTemp;

		// box association
		std::vector<std::deque<box3D>> lidarBoxesHistTemp;
		std::vector<int> bestMatch;
		std::deque<mapManager::box3D> newSingleBoxHist_;
		bestMatch.resize(this->lidarBoxes_.size(),-1);
		// very first frame, push boxes into boxes history
		if (this->lidarBoxesHist_.size() == 0){
			lidarBoxesHistTemp.resize(this->lidarBoxes_.size());
			for (size_t i=0 ; i<this->lidarBoxes_.size() ; ++i){
				lidarBoxesHistTemp[i].push_back(this->lidarBoxes_[i]);
			}
		}
		// perform box association
		else{
			// find best match index
			this->findBestMatch(bestMatch);

			// inherent previous history
			std::cout << "inhere previous history" <<std::endl;
			for (size_t i=0 ; i<this->lidarBoxes_.size() ; ++i){
				// if there is a match, inherent history
				if (bestMatch[i]){
					// inherent previous history
					lidarBoxesHistTemp.push_back(this->lidarBoxesHist_[bestMatch[i]]);
				}
				else { // otherwise, set empty history here
					lidarBoxesHistTemp.push_back(newSingleBoxHist_);
				}

				// pop out old data if the history is over the lenth require
				if (int(lidarBoxesHistTemp[i].size()) == histSize){
					lidarBoxesHistTemp[i].pop_back();
				}

				// push new data in the history
				lidarBoxesHistTemp[i].push_front(this->lidarBoxes_[i]);
			}
		}
        this->lidarBoxesHist_ = lidarBoxesHistTemp;

		// calc velocity
		for (size_t i=0 ; i<this->lidarBoxes_.size() ; ++i){
			this->lidarBoxes_[i].Vx = (this->lidarBoxesHist_[i][0].x - this->lidarBoxesHist_[i].back().x)/(dt*(this->lidarBoxesHist_.size()-1));
			this->lidarBoxes_[i].Vy = (this->lidarBoxesHist_[i][0].y - this->lidarBoxesHist_[i].back().y)/(dt*(this->lidarBoxesHist_.size()-1));
			std::cout << "box " << i <<" velocity: " <<this->lidarBoxes_[i].Vx << " " << this->lidarBoxes_[i].Vy << std::endl;
		}
        // 
    }

	void dynamicMapUGV::findBestMatch(std::vector<int> &bestMatch){
		std::cout<<"findBestMatch"<<endl;
		for (size_t i=0 ; i<this->lidarBoxes_.size() ; ++i){
			double minDist = 10;
			int bestMatchInd = -1;
			for (size_t j=0 ; j<this->lidarBoxesHist_.size() ; ++j){ // hist[i][0] is previous box[i]
				// calc centre distance
				double x_diff = this->lidarBoxes_[i].x-this->lidarBoxesHist_[j][0].x;
				double y_diff = this->lidarBoxes_[i].y-this->lidarBoxesHist_[j][0].y;
				double dist = std::sqrt(std::pow(x_diff, 2) + std::pow(y_diff , 2));

				if (dist<minDist){
					minDist = dist;
					bestMatchInd = j;
				}
			}

			bestMatch[i] = (minDist<=0.25)? bestMatchInd:-1;
		}
	}

	void dynamicMapUGV::getDynamicObstacles(std::vector<Eigen::Vector3d>& obstaclePos,
										 std::vector<Eigen::Vector3d>& obstacleVel,
										 std::vector<Eigen::Vector3d>& obstacleSize){

        // get uav boxes
		std::vector<mapManager::box3D> dynamicBBoxes;
		this->detector_->getDynamicObstacles(dynamicBBoxes);
		for (size_t i=0 ; i<dynamicBBoxes.size() ; ++i){
			Eigen::Vector3d pos(dynamicBBoxes[i].x, dynamicBBoxes[i].y, dynamicBBoxes[i].z);
			Eigen::Vector3d vel(dynamicBBoxes[i].Vx, dynamicBBoxes[i].Vy, 0);
			Eigen::Vector3d size(dynamicBBoxes[i].x_width, dynamicBBoxes[i].y_width, dynamicBBoxes[i].z_width);
			obstaclePos.push_back(pos);
			obstacleVel.push_back(vel);
			obstacleSize.push_back(size);
		}

        // keep lidar boxes out of UAV's FOV only
        for (size_t i=0 ; i<this->lidarBoxes_.size() ; ++i){
            if (this->isInFov(this->lidarBoxes_[i], this->position_, this->orientation_)){
                continue;
            }
            else{
                Eigen::Vector3d pos(this->lidarBoxes_[i].x, this->lidarBoxes_[i].y, this->lidarBoxes_[i].z);
                Eigen::Vector3d vel(this->lidarBoxes_[i].Vx, this->lidarBoxes_[i].Vy, 0);
                Eigen::Vector3d size(this->lidarBoxes_[i].x_width, this->lidarBoxes_[i].y_width, this->lidarBoxes_[i].z_width);
                obstaclePos.push_back(pos);
                obstacleVel.push_back(vel);
                obstacleSize.push_back(size);

            }
        }
	}

    bool dynamicMapUGV::isInFov(const box3D& box, const Eigen::Vector3d& position , const Eigen::Matrix3d& orientation){
        
        Eigen::Vector3d centerInCam;
        Eigen::Vector3d centerInWorld(box.x, box.y, box.z);
    
        centerInCam = orientation.inverse()*(centerInWorld-position);

        double x = centerInCam(0)-position(0);
        double y = centerInCam(1)-position(1);
        double z = std::sqrt(std::pow(y,2)+std::pow(x,2));
		std::cout << "isInFov: (x,z) :"<< x <<" " <<z <<std::endl;
        return x/z > cos(43.0*M_PI/180.0);

    }

	void dynamicMapUGV::publish3dBox(const std::vector<box3D>& boxes, const ros::Publisher& publisher, double r, double g, double b) {
        // visualization using bounding boxes 
        visualization_msgs::Marker line;
        visualization_msgs::MarkerArray lines;
        line.header.frame_id = "map";
        line.type = visualization_msgs::Marker::LINE_LIST;
        line.action = visualization_msgs::Marker::ADD;
        line.ns = "box3D";  
        line.scale.x = 0.03;
        line.color.r = r;
        line.color.g = g;
        line.color.b = b;
        line.color.a = 1.0;
        line.lifetime = ros::Duration(0.1);
        

        for(size_t i = 0; i < boxes.size(); i++){
			if (this->isInFov(boxes[i], this->position_, this->orientation)){ // only show lidar box when they are out of fov
				continue;
			}
            // visualization msgs
            line.text = " Vx " + std::to_string(boxes[i].Vx) + " Vy " + std::to_string(boxes[i].Vy);
            double x = boxes[i].x; 
            double y = boxes[i].y; 
            double z = (boxes[i].z+boxes[i].z_width/2)/2; 

            // double x_width = std::max(boxes[i].x_width,boxes[i].y_width);
            // double y_width = std::max(boxes[i].x_width,boxes[i].y_width);
            double x_width = boxes[i].x_width;
            double y_width = boxes[i].y_width;
            double z_width = 2*z;

            // double z = 
            
            vector<geometry_msgs::Point> verts;
            geometry_msgs::Point p;
            // vertice 0
            p.x = x-x_width / 2.; p.y = y-y_width / 2.; p.z = z-z_width / 2.;
            verts.push_back(p);

            // vertice 1
            p.x = x-x_width / 2.; p.y = y+y_width / 2.; p.z = z-z_width / 2.;
            verts.push_back(p);

            // vertice 2
            p.x = x+x_width / 2.; p.y = y+y_width / 2.; p.z = z-z_width / 2.;
            verts.push_back(p);

            // vertice 3
            p.x = x+x_width / 2.; p.y = y-y_width / 2.; p.z = z-z_width / 2.;
            verts.push_back(p);

            // vertice 4
            p.x = x-x_width / 2.; p.y = y-y_width / 2.; p.z = z+z_width / 2.;
            verts.push_back(p);

            // vertice 5
            p.x = x-x_width / 2.; p.y = y+y_width / 2.; p.z = z+z_width / 2.;
            verts.push_back(p);

            // vertice 6
            p.x = x+x_width / 2.; p.y = y+y_width / 2.; p.z = z+z_width / 2.;
            verts.push_back(p);

            // vertice 7
            p.x = x+x_width / 2.; p.y = y-y_width / 2.; p.z = z+z_width / 2.;
            verts.push_back(p);
            
            int vert_idx[12][2] = {
                {0,1},
                {1,2},
                {2,3},
                {0,3},
                {0,4},
                {1,5},
                {3,7},
                {2,6},
                {4,5},
                {5,6},
                {4,7},
                {6,7}
            };
            
            for (size_t i=0;i<12;i++){
                line.points.push_back(verts[vert_idx[i][0]]);
                line.points.push_back(verts[vert_idx[i][1]]);
            }
            
            lines.markers.push_back(line);
            
            line.id++;
        }
        // publish
        publisher.publish(lines);
    }

}

