#ifndef VISG_SLAM_H
#define VISG_SLAM_H
#include "camera.h"
#include "frame.h"
namespace VISG {
class VisgSlam {
public:
	VisgSlam();
	void Run(cv::Mat left=cv::Mat(),cv::Mat right = cv::Mat());
private:
	Camera zed_;
	cv::Mat left_, right_;

	Frame::Ptr p_cur_frame;
	Frame::Ptr p_last_frame;
};
}

#endif //visg_slam.h
