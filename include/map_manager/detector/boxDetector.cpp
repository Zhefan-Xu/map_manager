#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sstream>
#include <math.h>
#include <vector>
#include <time.h>
#include <map_manager/detector/uv_detector.h>
#include <map_manager/detector/kalman_filter.h>
#include <Eigen/Dense>
#include <queue>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_listener.h>

// #include <librealsense2/rs.hpp>



using namespace cv; 
using namespace std;
namespace mapManager{
	class boxDetector
	{   
		private:  
			queue<cv::Mat> depthq;
			cv::Mat depth;
			ros::NodeHandle nh_;   		// define node
			image_transport::Subscriber depsub;		// define subscriber for depth image
			image_transport::Subscriber imgsub;
			image_transport::Publisher depthBoxPub;
			image_transport::Publisher UmapBoxPub;

			UVdetector uvDetector_;
			std::vector<box3D> box3Ds;
			visualization_msgs::MarkerArray box3dMarkerLines;
			// ros::Publisher marker_pub;
			// ros::Publisher bboxes_pub;
			// ros::Publisher box_marker_pub;
			
			// tf::TransformListener listener;
			// tf2_ros::Buffer tfBuffer;
			// tf2_ros::TransformListener tfListener;

			double fx_;
			double fy_;
			double cx_;
			double cy_;

			double depthScale_;

			std::string ns_;


		public:  
			
			// boxDetector():tfListener(tfBuffer)  
			boxDetector(const ros::NodeHandle& nh, std::string ns, cv::Mat depth, double fx, double fy, double cx, double cy, double depthScale)
			{  	
				this->depth = depth;
				this->fx_ = fx;
				this->fy_ = fy;
				this->cx_ = cx;
				this->cy_ = cy;
				this->depthScale_ = depthScale;
				this->uvDetector_.fx = fx;
				this->uvDetector_.fy = fy;
				this->uvDetector_.px = cx;
				this->uvDetector_.py = cy;
				this->uvDetector_.depthScale_ = this->depthScale_;

				this->nh_ = nh;
				this->ns_ = ns;
				image_transport::ImageTransport it(nh);
				this->depthBoxPub = it.advertise(this->ns_+"/uv/depth", 1);
				this->UmapBoxPub = it.advertise(this->ns_+"/uv/Umap", 1);

			}  
  
			void getDepth(cv::Mat &depth) {
				depth = this->depth;
			}

			void getBox3Ds(std::vector<box3D> &incomeBox3Ds) 
			{

				incomeBox3Ds = this->box3Ds;
			}

			void getBox3dMarkerLines(visualization_msgs::MarkerArray &markers) {
				markers = this->box3dMarkerLines;
			}

			void getBox2Ds(std::vector<Rect> &box2Ds){
				box2Ds = this->uvDetector_.bounding_box_D;
			}

			void getBoxBirds(std::vector<Rect> &boxBirds) {
				boxBirds = this->uvDetector_.bounding_box_B;
			}
   
		
			void getBBox(const cv::Mat &depth, Eigen::Vector3d &position, Eigen::Matrix3d &orientation){
				this->depth = depth;
				if (!this->depth.empty()){
					this->uvDetector_.depth = this->depth;
					this->uvDetector_.detect();
					
					this->uvDetector_.display_U_map();
					this->uvDetector_.display_bird_view();

					this->uvDetector_.extract_3Dbox();
					this->uvDetector_.track();
					this->uvDetector_.display_depth();
					// publish depth/umap detection vis
					sensor_msgs::ImagePtr depthBoxMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", this->uvDetector_.depth_show).toImageMsg();
					sensor_msgs::ImagePtr UmapBoxMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", this->uvDetector_.U_map_show).toImageMsg();
					this->depthBoxPub.publish(depthBoxMsg);
					this->UmapBoxPub.publish(UmapBoxMsg);
					this->uvDetector_.box3DsWorld.clear();

					Eigen::Vector3d currPointCam, currPointMap;
					for(size_t i = 0; i < this->uvDetector_.box3Ds.size(); i++){

						// visualization msgs
						box3D box;
						box.Vx = 0.;
						box.Vy = 0.;

						float x = this->uvDetector_.box3Ds[i].x; 
						float y = this->uvDetector_.box3Ds[i].y;
						float z = this->uvDetector_.box3Ds[i].z;
						currPointCam(0) = x;
						currPointCam(1) = y;
						currPointCam(2) = z;
						currPointMap = orientation * currPointCam + position; // transform to map coordinate
						// printf("scale %d\n",this->depthScale_);
						// printf("3d box %d on camera\n: %f, %f, %f,%f ,%f, %f\n",i,x,y,z,x_width,y_width,z_width);
						// std::cout<<position<<std::endl;
						// std::cout<<orientation<<std::endl;
						box.x = currPointMap(0);
						box.y = currPointMap(1);
						box.z = currPointMap(2);
						box.x_width = this->uvDetector_.box3Ds[i].x_width;
						box.y_width = this->uvDetector_.box3Ds[i].z_width;
						box.z_width = this->uvDetector_.box3Ds[i].y_width;
						// printf("3d box %d on world from raw detector\n: %f, %f, %f,%f ,%f, %f\n",i,box.x,box.y,box.z,box.x_width,box.y_width,box.z_width);
						this->uvDetector_.box3DsWorld.push_back(box);
						
					}
					this->box3Ds = this->uvDetector_.box3DsWorld;

				}
				else {
					ROS_INFO("did not get depth");
					this->uvDetector_.bounding_box_D.clear();
					this->uvDetector_.bounding_box_B.clear();
				}
			}

		

	};
}
