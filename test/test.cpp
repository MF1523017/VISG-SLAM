#include "test.h"
#include "feature.h"
#include "frame.h"
#include "draw_board.h"
#include "utils.hpp"
#include "camera.h"
#include "visg_slam.h"
#include "timer.h"
#include <thread>
using namespace VISG;

void test_Camera(){
	Camera zed;
	if (!zed.Open(2,60)) {
		std::cout << "zed open error." << std::endl;
		return;
	}
	
	zed.SetCameraSettings(4, 4, 0, 5, 80, 80, 3900);

	std::cout << zed.cam_info << std::endl << zed.cam_info_raw << std::endl;
	cv::Mat left, right;
	Frame current;
	while (true) {
		if (zed.Grab(left, right)) {
			//cv::imshow("image", left);
			//imshow("image", left, right);
			cv::cvtColor(left, left, CV_RGBA2GRAY);
			cv::cvtColor(right, right, CV_RGBA2GRAY);
			current.ExtractFeatures(left, right);
			std::cout << "fps: " << zed.GetFPS() << std::endl;
		}
	}
	zed.Close();
}

void test_VisgSlam() {
	VisgSlam visg;
	visg.Run();
}

void test_VisgSlam(char **argv) {
	VisgSlam visg;
	const std::string file_dir(argv[2]);
	visg.RecordImages(file_dir);
}

void test_VisgSlamOffline(char **argv){
	VisgSlamOffline visg;
	std::vector<std::string> images;
	const std::string data_dir(argv[1]);
	loadImage(data_dir,images);
	for (size_t i = 0; i < images.size(); ++i) {
		const std::string left_image(data_dir + "\\cam0\\data\\" + images[i]);
		const std::string right_image(data_dir + "\\cam1\\data\\" + images[i]);
		cv::Mat left = cv::imread(left_image);
		cv::Mat right = cv::imread(right_image);
		if (left.empty() || right.empty()) {
			std::cout << "error" << std::endl;
			return;
		}
		visg.Run(left, right);
	}
}


void test_offline() {
	VisgSlamOffline visg;
	std::vector<std::string> images;
	//const std::string data_dir("H:\\dataset\\20171207_demo\\20171207");
	//const std::string data_dir("H:\\dataset\\20171120\\20171120");
	const std::string data_dir("H:\\dataset\\20171214_1\\20171214");
	loadImage(data_dir, images);
	Timer timer;
	for (size_t i = 0; i < images.size(); ++i) {
		timer.Reset();
		const std::string left_image(data_dir + "\\cam0\\data\\" + images[i]);
		const std::string right_image(data_dir + "\\cam1\\data\\" + images[i]);
		cv::Mat left = cv::imread(left_image);
		cv::Mat right = cv::imread(right_image);
		if (left.empty() || right.empty()) {
			std::cout << "error" << std::endl;
			return;
		}
		visg.Run(left, right);
		std::cout << "Time elapsed(ms): " << timer.ElapsedMS() << std::endl;
	}
}
