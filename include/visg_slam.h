#ifndef VISG_SLAM_H
#define VISG_SLAM_H
#include "camera.h"
#include "orb_tracker.h"
#include "chessboard.h"
//#define HDMODE
namespace VISG {
class VisgSlam {
public:
	VisgSlam();
	void RecordImages(const std::string &data_dir);
	void Run(cv::Mat left=cv::Mat(),cv::Mat right = cv::Mat());
private:
	Camera zed_;
	cv::Mat left_, right_;
	TrackerInterface::Ptr tracker_;
};

class VisgSlamOffline {
public:
	VisgSlamOffline();
	void Run(cv::Mat &left, cv::Mat &right);
	
private:
	TrackerInterface::Ptr tracker_;
};
}

#endif //visg_slam.h
