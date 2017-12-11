#ifndef __UTILS_HPP__
#define __UTILS_HPP__
#include <math.h>
#include <stdint.h>
#include <vector>
#include <string>
#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>
#include "type.h"
#include "common.h"

#ifndef M_PI
#define M_PI 3.1416f
#endif
namespace VISG {
	// convert sl Mat to cv Mat
	cv::Mat slMat2cvMat(sl::Mat& input);
	// show left and right pic
	void imshow(const std::string &name, const cv::Mat &left,const cv::Mat &right);
	// convert depth to image to save
	void depth2Image(const cv::Mat& depth, cv::Mat &depth_image);

	Eigen::Matrix3f Rcv2Eigen(const cv::Mat &R);
	Eigen::Vector3f Tcv2Eigen(const cv::Mat &t);
	Eigen::Vector3f Pcv2Eigen(const cv::Point3f &p);
	cv::Point3f PEigen2cv(const Eigen::Vector3f &p);
	Eigen::Vector3f R2ypr(const Eigen::Matrix3f &R);
	Eigen::Matrix4f HPose(const Eigen::Matrix3f &R, const Eigen::Vector3f &t);
	void HPose2Rt(const Eigen::Matrix4f &T, Eigen::Matrix3f &R, Eigen::Vector3f &t);
	void loadImage(const std::string & file_dir, std::vector<std::string> &images);
}

#endif /*__UTILS_HPP__*/
