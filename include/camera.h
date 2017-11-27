#ifndef CAMERA_H
#define CAMERA_H
#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>
namespace VISG {
	class Camera {
	public:
		/*
		* @brief: resolution: 0: 4k,< 2208*1242, available framerates: 15 fps>
							  1:1080,< 1920*1080, available framerates: 15, 30 fps>
							  2: 720,< 1280*720, available framerates: 15, 30, 60 fps>
							  3:vga < 672*376, available framerates: 15, 30, 60, 100 fps>
		*/
		bool Open(int resolution = 2, int fps = 60);
		/*
		* @brief: brightness:  Defines the brightness control. Affected value should be between 0 and 8.
				  contrast:	Defines the contrast control. Affected value should be between 0 and 8.
				  hue: Defines the hue control. Affected value should be between 0 and 11.
				  saturation: Defines the saturation control. Affected value should be between 0 and 8.
				  gain: Defines the gain control. Affected value should be between 0 and 100 for manual control. If ZED_EXPOSURE is set to -1, the gain is in auto mode too.
				  exposure: Defines the exposure control. A -1 value enable the AutoExposure/AutoGain control,as the boolean parameter (default) does. Affected value should be between 0 and 100 for manual control.
				  whitebalance:  Defines the color temperature control. Affected value should be between 2800 and 6500 with a step of 100. A value of -1 set the AWB ( auto white balance), as the boolean parameter (default) does.
		*/
		void SetCameraSettings(unsigned int brightness = 4, unsigned int contrast = 4, unsigned int hue = 0, unsigned int saturation = 4,
			unsigned int gain = 90, int exposure = -1, int whiteblance = -1);
		/*
		* @brief: grab left and right image and convert sl::Mat to cv::Mat
		*/
		bool Grab(cv::Mat &left, cv::Mat &right);
		/*
		* @brief: close zed camera
		*/
		void Close() {
			zed_.close();
		}
		/*
		* @brief: get camera current timestamp
		*/
		unsigned long long GetTimestamp() {
			return zed_.getCameraTimestamp();
		}
		/*
		* @brief: get camera current fps
		*/
		float GetFPS() {
			//return zed_.getCameraFPS();
			return zed_.getCurrentFPS();
		}
	private:
		sl::Camera zed_;// zed instance
		sl::Mat left_; // left image
		sl::Mat right_;// right image
	};



}


#endif //  camera.h
