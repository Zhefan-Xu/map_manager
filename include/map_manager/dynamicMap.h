/*
	FILE: dynamicMap.h
	--------------------------------------
	header file dynamic map
*/

#ifndef MAPMANAGER_DYNAMICMAP_H
#define MAPMANAGER_DYNAMICMAP_H

#include <map_manager/occupancyMap.h>
#include <map_manager/dynamicDetector.h>
#include <nav_msgs/Path.h>

#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <unsupported/Eigen/MatrixFunctions>

// using namespace cv; 
// using namespace std;
namespace mapManager{

	class dynamicMap : public occMap{
	// private:
	protected:
		ros::NodeHandle nh_;

		// PARAMS
		// -----------------------------------------------------------------
		int ts_;
		// ROS
		ros::Timer dynamicObsDetectTimer_;
		ros::Timer visTimer_;

		// detector
		std::shared_ptr<mapManager::dynamicDetector> detector_;

		// publisher
		ros::Publisher depthCloudFilteredPub_;
		// ros::Publisher depthCloudPub_;
	public:
		dynamicMap();
		dynamicMap(const ros::NodeHandle& nh);

		// init
		void initMap(const ros::NodeHandle& nh);
		void initDynamicParam();
		void registerCallback(); // re-write occmap callback
		void registerDynamicPub();
		void registerDynamicCallback();

		// callback
		void visDynamicCB(const ros::TimerEvent&);
		void inflateMapCB(const ros::TimerEvent&);
		void updateOccupancyCB(const ros::TimerEvent&);
		void dynamicObsDetectCB(const ros::TimerEvent&);
		// void inflateMapCB(const ros::TimerEvent&);
		// void updateOccupancyCB(const ros::TimerEvent&);
		// void boxDetectCB(const ros::TimerEvent&);
		// void obstacleTrajPubCB(const ros::TimerEvent&);
		// void dynamicBoxPubCB(const ros::TimerEvent&);

		// publish
		void publishFilteredPoinCloud();
	
		
	

	};

	// template <typename T> 
	// inline T dynamicMap::norm(const T &x, const T &y){
	// 	return std::sqrt(std::pow(x,2)+std::pow(y,2));
	// }

	// template <typename T> 
	// inline T dynamicMap::distance(const T &Ax, const T &Ay, const T &Bx, const T &By){
	// 	return norm<T>(Ax-Bx, Ay-By);
	// }

	// template <typename T> 
	// inline void dynamicMap::initTrackedStatesArr(T &preContainer, T &nowContainer, const int size){
	// 	preContainer = nowContainer;
	// 	nowContainer.clear();
	// 	nowContainer.resize(size);
	// }

	// template <typename T> 
	// inline void dynamicMap::initTrackedStatesArr(T &container, const int size){
	// 	container.clear();
	// 	container.resize(size);
	// }

	// template <typename T, typename U> 
	// inline void dynamicMap::initTrackedStatesVar(T &preVar, T &nowVar, U &income){
	// 	preVar = nowVar;
	// 	nowVar = income;
	// }

	// inline bool dynamicMap::isInFov(const Rect &box2d) {
	// 	return (box2d.tl().x>this->fovXMarginLower_ && box2d.br().x <this->fovXMarginUpper_ && box2d.tl().y >this->fovYMarginLower_ && box2d.br().y <this->fovYMarginUpper_ );
	// }	

}

#endif

