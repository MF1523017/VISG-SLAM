#ifndef __UTILS_HPP__
#define __UTILS_HPP__

#define NDEBUG



#include <math.h>
#include <stdint.h>
#include <vector>
#include <string>
#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>


#ifndef M_PI
#define M_PI 3.1416f
#endif
// sl2cv

namespace VISG {
	// convert sl Mat to cv Mat
	cv::Mat slMat2cvMat(sl::Mat& input);
	// show left and right pic
	void imshow(const std::string &name, const cv::Mat &left,const cv::Mat &right);
	// convert depth to image to save
	void depth2Image(const cv::Mat& depth, cv::Mat &depth_image);
	/*
	*/

}

#endif /*__UTILS_HPP__*/
