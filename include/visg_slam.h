#ifndef VISG_SLAM_H
#define VISG_SLAM_H
#include "camera.h"
#include "orb_tracker.h"
#include "chessboard.h"

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
	VisgSlamOffline(const std::string &dictionary);
	void Run(cv::Mat &left, cv::Mat &right);
	void SaveMapPoints(const std::string &file_name);
	// for test
	std::vector<Eigen::Vector3f> positions_groundtruth()const { return positions_groundtruth_; }
	std::vector<Eigen::Vector3f> positions_slam()const { return positions_slam_; }
	std::vector<Eigen::Vector3f> positions_key_frame();

private:
	TrackerInterface::Ptr tracker_;
	//for test
	std::vector<Eigen::Vector3f> positions_groundtruth_;
	std::vector<Eigen::Vector3f> positions_slam_;
	std::vector<Eigen::Vector3f> positions_key_frame_;
};
}

#endif //visg_slam.h
