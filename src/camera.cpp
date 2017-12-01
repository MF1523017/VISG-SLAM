#include "camera.h"
#include "utils.hpp"
namespace VISG{

Intrinsic::Intrinsic(const sl::CameraParameters &rhs){
	*this = rhs;
}

Intrinsic & Intrinsic::operator=(const sl::CameraParameters &rhs) {
	fx = rhs.fx;
	fy = rhs.fy;
	cx = rhs.cx;
	cy = rhs.cy;
	for (int i = 0; i < 5; ++i) {
		disto[i] = rhs.disto[i];
	}
	v_fov = rhs.v_fov;
	h_fov = rhs.h_fov;
	d_fov = rhs.d_fov;
	image_size.height = rhs.image_size.height;
	image_size.width = rhs.image_size.width;
	return *this;
}

CameraInfo::CameraInfo(const sl::CalibrationParameters &rhs) {
	*this = rhs;
}
CameraInfo & CameraInfo::operator = (const sl::CalibrationParameters &rhs) {
	cv::Mat R_vec(3, 1, CV_32FC1, (void*)rhs.R.ptr());
	cv::Rodrigues(R_vec, extrinsic.R);
	extrinsic.t.at<float>(0,0) = rhs.T.x;
	extrinsic.t.at<float>(1,0) = rhs.T.y;
	extrinsic.t.at<float>(2,1) = rhs.T.z;
	left_cam = rhs.left_cam;
	right_cam = rhs.right_cam;
	return *this;
}
std::ostream & operator <<(std::ostream &os, const CameraInfo &rhs) {
	os << "Extrinsic: " << std::endl << " R: " << rhs.extrinsic.R << std::endl <<
		"t: " << rhs.extrinsic.t << std::endl <<
		"Intrinsic: " << std::endl << "left cam: " << std::endl <<
		"fx: " << rhs.left_cam.fx << " fy: " << rhs.left_cam.fy << " cx: " << rhs.left_cam.cx << " cy: " << rhs.left_cam.cy <<
		std::endl << "right cam: " << std::endl <<
		"fx: " << rhs.right_cam.fx << " fy: " << rhs.right_cam.fy << " cx: " << rhs.right_cam.cx << " cy: " << rhs.right_cam.cy;
	return os;
}

bool Camera::Open(int resolution , int fps ) {
	sl::InitParameters init_parameters;
	init_parameters.camera_resolution = sl::RESOLUTION(resolution);
	init_parameters.camera_fps = fps;
	sl::ERROR_CODE err = zed_.open(init_parameters);
	if (err != sl::SUCCESS) {
		std::cout << sl::errorCode2str(err) << std::endl;
		return false;
	}
	cam_info = zed_.getCameraInformation().calibration_parameters;
	cam_info_raw = zed_.getCameraInformation().calibration_parameters_raw;
	return true;
}

void Camera::SetCameraSettings(unsigned int brightness, unsigned int contrast, unsigned int hue , unsigned int saturation ,
	unsigned int gain , int exposure , int whiteblance) {
	zed_.setCameraSettings(sl::CAMERA_SETTINGS_BRIGHTNESS, brightness, false);
	zed_.setCameraSettings(sl::CAMERA_SETTINGS_CONTRAST, contrast, false);
	zed_.setCameraSettings(sl::CAMERA_SETTINGS_HUE, hue, false);
	zed_.setCameraSettings(sl::CAMERA_SETTINGS_SATURATION, saturation, false);
	zed_.setCameraSettings(sl::CAMERA_SETTINGS_GAIN, gain, false);
	zed_.setCameraSettings(sl::CAMERA_SETTINGS_EXPOSURE, exposure, false);
	zed_.setCameraSettings(sl::CAMERA_SETTINGS_WHITEBALANCE, whiteblance, false);
}

bool Camera::Grab(cv::Mat &left, cv::Mat &right) {
	if (zed_.grab() == sl::SUCCESS) {
		zed_.retrieveImage(left_, sl::VIEW_LEFT);
		zed_.retrieveImage(right_, sl::VIEW_RIGHT);
		left = slMat2cvMat(left_);
		right = slMat2cvMat(right_);
		return true;
	}
	return false;
}

}