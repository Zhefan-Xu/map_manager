/*
	FILE: occupancyMap.cpp
	--------------------------------------
	function definition of occupancy map
*/
#include <map_manager/occupancyMap.h>

namespace mapManager{
	occMap::occMap(){}

	void occMap::initMap(const ros::NodeHandle& nh){
		this->nh_ = nh;
		this->initParam();
		this->initCallback();
	}

	void occMap::initParam(){
		// depth topic name
		if (not this->nh_.getParam("occupancy_map/depth_image_topic", this->depthTopicName_)){
			this->depthTopicName_ = "/camera/depth/image_raw";
			cout << "[OccMap]: No depth image topic name. Use default: /camera/depth/image_raw" << endl;
		}
		else{
			cout << "[OccMap]: Depth topic: " << this->depthTopicName_ << endl;
		}

		// pose topic name
		if (not this->nh_.getParam("occupancy_map/pose_topic", this->poseTopicName_)){
			this->poseTopicName_ = "/CERLAB/quadcopter/pose";
			cout << "[OccMap]: No pose topic name. Use default: /CERLAB/quadcopter/pose" << endl;
		}
		else{
			cout << "[OccMap]: Pose topic: " << this->poseTopicName_ << endl;
		}

	}

	void occMap::initCallback(){
		// depth pose callback
		this->depthSub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(this->nh_, this->depthTopicName_, 50));
		this->poseSub_.reset(new message_filters::Subscriber<geometry_msgs::PoseStamped>(this->nh_, this->poseTopicName_, 25));
		this->depthPoseSync_.reset(new message_filters::Synchronizer<depthPoseSync>(depthPoseSync(100), *this->depthSub_, *this->poseSub_));
		this->depthPoseSync_->registerCallback(boost::bind(&occMap::depthPoseCallback, this, _1, _2));
		// occupancy update callback
		
		// visualization callback
	}


	void occMap::depthPoseCallback(const sensor_msgs::ImageConstPtr& img, const geometry_msgs::PoseStampedConstPtr& pose){
		cout << "depth pose callback trigger" << endl;
	}
}