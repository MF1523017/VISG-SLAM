#include "common.h"
namespace VISG {
	size_t Common::Height = 0;
	size_t Common::Weight = 0;
	float Common::Fx = .0;
	float Common::Fy = .0;
	float Common::Cx = .0;
	float Common::Cy = .0;
	cv::Mat Common::lRr = cv::Mat::eye(3, 3, CV_32FC1);
	cv::Mat Common::ltr = cv::Mat::zeros(3, 1, CV_32FC1);
	size_t Common::HistBin = 200;
}