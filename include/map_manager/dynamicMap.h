/*
	FILE: dynamicMap.h
	--------------------------------------
	header file dynamic map
*/

#ifndef MAPMANAGER_DYNAMICMAP_H
#define MAPMANAGER_DYNAMICMAP_H

#include <map_manager/occupancyMap.h>
#include <nav_msgs/Path.h>
#include <map_manager/detector/boxDetector.cpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <unsupported/Eigen/MatrixFunctions>

// using namespace cv; 
// using namespace std;
namespace mapManager{

	class dynamicMap : public occMap{
	private:
		ros::NodeHandle nh_;

		// PARAMS
		// -----------------------------------------------------------------

		// debug
		bool detectionDebug_;
		bool veDebug_;

		// test
		bool testVel_;
		bool testPos_;

		// map refinement
		float mapRefineInflateCoef_;
		float rawBoxWidthLimitLower_;
		float rawBoxHeightLimitLower_;
		float rawBoxWidthLimitUpper_;
		float rawBoxHeightLimitUpper_;
		float mapRefineFloor_;

		// dynamicRegionClean
		int histCleanSkip_;
		float cleanExtraDist_;
		float groundThick_;

		// trackAndEstimateVelocity
		double ts_;
		float overlapThreshold_; // threshold to determind tracked or not
		float contTresh_;
		float vAvgEstimatedMetric_;

		// trackAndIdentifyDynamic
		// updateTrackedStates
		int fovXMarginLower_;
		int fovXMarginUpper_;
		int fovYMarginLower_;
		int fovYMarginUpper_;

		// updateNonTrackedStates
		// kalman filter
		double eP_,eQ_,eR_;

		// identifyDynamicObjs
		long unsigned int preHistCleanSize_;

		// firstLayerDetect
		long unsigned int avgVHistSize_;
		long unsigned int nowHistSize_;
		long unsigned int CFHistSize_;
		long unsigned int CFInterval_;
		float sizeRatioUpper_;
		float sizeRatioLower_;
		float fusedBoxWidthLimitLower_;
		float fusedBoxHeightLimitLower_;
		float fusedBoxWidthLimitUpper_;
		float fusedBoxHeightLimitUpper_;
		float lowestPointLimit_;
		

		// secondLayerDetect
		int staticThresh_;
		int dynamicThresh_;
		long unsigned int voteHistSize_;

		// trajectory prediction
		int n_;
		double t_;
		double colliCheckEndDist_;
		double colliCheckStartDist_;
		MatrixXd PT_; // state transformation
		MatrixXd PI_; // initial state


		// U_DEPTH_MAP_DETECTOR
		// -----------------------------------------------------------------
		ros::Timer boxDetectTimer_;
		ros::Timer dynamicBoxPubTimer_;
		ros::Timer obstacleTrajPubTimer_;
		std::shared_ptr<mapManager::boxDetector> detector_;
		std::vector<box3D> rawBox3Ds_;
		std::vector<Rect> box2Ds_;
		std::vector<Rect> boxBirds_;
		visualization_msgs::MarkerArray box3dRawMarkerLines;



		// MAP REFINEMENT
		// -----------------------------------------------------------------
		std::vector<box3D> fusedBox3Ds_;
		std::vector<box3D> dynamicObjs_;



		// TRACK & ESTIMATE VELOCITY
		// -----------------------------------------------------------------
		bool getEmpty_;
		std::vector<box3D> preBb_;// bbox. Real value after fixing
		std::vector<box3D> nowBb_;
		std::vector<cv::Rect> nowBb2d_;
		std::vector<cv::Rect> preBb2d_;
		std::vector<box3D> nowOrg_;// bbox original value before fixing
		std::vector<box3D> preOrg_;
		std::vector<bool> preDynamic_;
		std::vector<bool> nowDynamic_;
		std::deque<std::deque<int>> preVoteHist_;
		std::deque<std::deque<int>> nowVoteHist_;	
		std::deque<std::deque<box3D>> nowHistInFov_; // bbox history that are in the FOV
    	std::deque<std::deque<box3D>> preHistInFov_;	
		std::deque<std::deque<cv::Rect>> now2dHist_;	
		std::deque<std::deque<cv::Rect>> pre2dHist_;	
		std::deque<std::deque<box3D>> nowHist_; // history that does not care whether in/out of FOV
    	std::deque<std::deque<box3D>> preHist_;
		std::vector<Eigen::MatrixXd> preKfStates_; // states includes x, y, vx, vy, width, depth
   		std::vector<Eigen::MatrixXd> nowKfStates_;
		std::vector<Eigen::MatrixXd> preKfP_; 
   		std::vector<Eigen::MatrixXd> nowKfP_;		
		std::deque<std::deque<MatrixXd>> preV_;// track the sum of predicted velocity for calculating avg
		std::deque<std::deque<MatrixXd>> nowV_;	 
		std::vector<bool> preFix_;// flag to fix box size
		std::vector<bool> nowFix_;
		std::vector<int> dynamicHistInd_; // index of dynamicObjs in fusedObjs trackhistory
		std::vector<box3D> fixCandidates_; // stores candidates of fixed box size. generated from pre_history
		




		// KALMAN FILTER
		// -----------------------------------------------------------------
		
		// Eigen::MatrixXd states_;
		Eigen::MatrixXd A_;
		Eigen::MatrixXd H_;
		// Eigen::MatrixXd P_;
		Eigen::MatrixXd Q_;
		Eigen::MatrixXd R_;


		// DYNAMIC REGION CLEAN
		// -----------------------------------------------------------------
		// std::vector<double> occupancyDynamicCopy_; // occupancy log data
		std::set<int> occAddressBeforeClean_;
		// std::vector<double> occupancyDynamic_;
		std::deque<std::deque<box3D>> nowHistClean_;
		std::deque<std::deque<box3D>> preHistClean_;





		// TRAJ PRED
		// -----------------------------------------------------------------
		
		std::vector<MatrixXd> coefs_;
		std::vector<std::vector<int>> endPoints_; // idx of point of collision
		// std::vector<std::vector<std::vector<geometry_msgs::Point>>> trajs;
		std::vector<std::vector<std::vector<geometry_msgs::Point>>>trajs_;
		std::vector<std::vector<std::vector<geometry_msgs::Point>>>trajsForColliCheck_;
		std::vector<std::vector<geometry_msgs::Point>> bestTrajs_;
		// markove track
		double overlapThreshM_;
		std::vector<int> resultInd_; // index of best traj
		std::vector<box3D> preBbMarkov_;
		std::vector<box3D> nowBbMarkov_;
		std::vector<MatrixXd> PS_; // social force probability
		std::vector<MatrixXd> PPre_; // previous state estimate probability
		std::vector<MatrixXd> PNow_; // current state estimate probability
		std::vector<MatrixXd> POut_; // output probability



		// PUBLISH
		ros::Publisher dynamicBoxPub_;
		ros::Publisher fusedBoxPub_;
		ros::Publisher rawBoxPub_;
		ros::Publisher obstacleTrajPub_;
		ros::Publisher dynamicVelPub_;
		ros::Publisher dynamicPosPub_;

	public:
		dynamicMap();

		// init
		void initMap(const ros::NodeHandle& nh);
		void initDynamicParam();
		void registerDynamicPub();
		void registerDynamicCallback();
		void registerCallback();

		// call back
		void inflateMapCB(const ros::TimerEvent&);
		void updateOccupancyCB(const ros::TimerEvent&);
		void boxDetectCB(const ros::TimerEvent&);
		void obstacleTrajPubCB(const ros::TimerEvent&);
		void dynamicBoxPubCB(const ros::TimerEvent&);

		// dynamic region clean
		void inflateLocalMap(const std::deque<std::deque<box3D>> &nowHistOut, const std::vector<int> &dynamicHistInd, const std::vector<box3D> &dynamicObjs);
		void dynamicRegionClean(const std::deque<std::deque<box3D>> &nowHistClean, const std::vector<int> &dynamicHistInd, const std::vector<box3D> &dynamicObjs);
		void cleanSingleFrame(const box3D &dynamicObj);

		// map refinement and dynamic detection
		void mapRefine();
		
		// track and estimate velocity
		void trackAndEstimateVe(std::vector<box3D> &dynamicObjs);
		void trackAndIdentifyDynamic();
		void identifyDynamicObjs(std::vector<box3D> &dynamicObjs, std::vector<int> &dynamicHistInd);
		void initTrack(const std::vector<box3D> &nowBb, const std::vector<cv::Rect> &nowBb2d );
		template <typename T> void initTrackedStatesArr(T &preContainer, T &nowContainer, const int size);// T must be a container
		template <typename T> void initTrackedStatesArr(T &container, const int size);// T must be a container
		template <typename T, typename U> inline void initTrackedStatesVar(T &preVar, T &nowVar, U &income);
		void storeFixCandSize();
		template <typename T> void keepHistSize(T &HistContainer, int size);
		void updateTrackedStates(const int &nowId, const int &preId);
		void updateNonTrackedStates(const int &nowId);
		void firstLayerDetect(std::vector<bool> &isDynamic);
		void secondLayerDetect(std::vector<bool> &isDynamic,std::vector<box3D> &dynamicObjs, std::vector<int> &dynamicHistInd);
		double continuityFilter(const int &nowId);
		void initKf(Eigen::MatrixXd &statesNowId, Eigen::MatrixXd &pNowId, const box3D &boxNowId);
		void kfEstimate(Eigen::MatrixXd &UpdateStates, Eigen::MatrixXd &UpdateP, const Eigen::MatrixXd &z, const Eigen::MatrixXd &states, const Eigen::MatrixXd &P);

		// traj pred
		void predictTrajectory();
		void generateTraj(const std::vector<box3D> &objs);
		void checkCollisionPos();
		void readBoxInfoMarkov(const std::vector<box3D> &nowBbMarkov, const std::vector<std::vector<int>> &endPoints);
		void predict();
	
		
	

		// toolbox functions
		template <typename T> T norm(const T &x, const T &y);
		template <typename T> T distance(const T &Ax, const T &Ay, const T &Bx, const T &By);
		bool isInFov(const Rect &box2d) ;
		bool checkTrack(const box3D &now, const box3D &pre);
		bool isCopyOccupied(const Eigen::Vector3d& pos);
		bool isCopyOccupied(const Eigen::Vector3i& idx); // does not count for unknown


		// user interface
		void getPredTraj(std::vector<std::vector<std::vector<geometry_msgs::Point>>>& predTraj);
		void getObjPos(std::vector<Eigen::Vector3d> &obstaclesPos);
		void getObjVel(std::vector<Eigen::Vector3d> &obstaclesVel);
		void getObjSize(std::vector<Eigen::Vector3d> &obstaclesSize);

		// publish
		void publish3dBox(const std::vector<box3D> &boxes, const ros::Publisher &publisher, const char &color);
		void publishVelAndPos(const std::vector<box3D> &boxes);
		void publishAllTraj();
		void publishBestTraj();
		// 
	};

	template <typename T> 
	inline T dynamicMap::norm(const T &x, const T &y){
		return std::sqrt(std::pow(x,2)+std::pow(y,2));
	}

	template <typename T> 
	inline T dynamicMap::distance(const T &Ax, const T &Ay, const T &Bx, const T &By){
		return norm<T>(Ax-Bx, Ay-By);
	}

	template <typename T> 
	inline void dynamicMap::initTrackedStatesArr(T &preContainer, T &nowContainer, const int size){
		preContainer = nowContainer;
		nowContainer.clear();
		nowContainer.resize(size);
	}

	template <typename T> 
	inline void dynamicMap::initTrackedStatesArr(T &container, const int size){
		container.clear();
		container.resize(size);
	}

	template <typename T, typename U> 
	inline void dynamicMap::initTrackedStatesVar(T &preVar, T &nowVar, U &income){
		preVar = nowVar;
		nowVar = income;
	}

	inline bool dynamicMap::isInFov(const Rect &box2d) {
		return (box2d.tl().x>this->fovXMarginLower_ && box2d.br().x <this->fovXMarginUpper_ && box2d.tl().y >this->fovYMarginLower_ && box2d.br().y <this->fovYMarginUpper_ );
	}	

}

#endif

