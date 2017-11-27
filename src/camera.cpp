#include "camera.h"
#include "utils.hpp"
namespace VISG{

bool Camera::Open(int resolution , int fps ) {
	sl::InitParameters init_parameters;
	init_parameters.camera_resolution = sl::RESOLUTION(resolution);
	init_parameters.camera_fps = fps;
	sl::ERROR_CODE err = zed_.open(init_parameters);
	if (err != sl::SUCCESS) {
		std::cout << sl::errorCode2str(err) << std::endl;
		return false;
	}
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