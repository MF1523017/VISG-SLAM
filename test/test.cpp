#include "test.h"
#include "feature.h"
#include "frame.h"
#include "draw_board.h"
#include "utils.hpp"
#include "camera.h"
#include "visg_slam.h"
#include <thread>
using namespace VISG;

void test_Camera(){
	// Create a ZED camera object
	Camera zed;
	//Feature * fp = new OrbFeature;
	// Open the camera
	if (!zed.Open(2,60)) {
		std::cout << "zed open error." << std::endl;
		return;
	}
	

	zed.SetCameraSettings(4, 4, 0, 5, 80, 80, 3900);

	// Capture 50 frames and stop
	int i = 0;
	std::cout << zed.cam_info << std::endl << zed.cam_info_raw << std::endl;
	cv::Mat left, right;
	Frame current;
	while (i < 50) {
		// Grab an image
		if (zed.Grab(left, right)) {
			//cv::imshow("image", left);
			//imshow("image", left, right);
			cv::cvtColor(left, left, CV_RGBA2GRAY);
			cv::cvtColor(right, right, CV_RGBA2GRAY);
			current.ExtractFeatures(left, right);
			std::cout << "fps: " << zed.GetFPS() << std::endl;
		/*	int key = cv::waitKey(5);
			if (key == 27)
				break;*/

		}
		
	}
	zed.Close();
	system("pause");
}

void test_VisgSlam() {
	VisgSlam visg;
	visg.Run();
}

void test_VisgSlam(char **argv) {
	VisgSlam visg;
	//visg.Run();
	const std::string file_dir(argv[1]);
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
#ifdef USE_CHESSBOARD
	const std::string data_dir("H:\\dataset\\20171207_chessboard\\20171207");
#else
	const std::string data_dir("H:\\dataset\\20171207_demo\\20171207");
	//const std::string data_dir("H:\\dataset\\20171120\\20171120");
#endif
	loadImage(data_dir, images);
	for (size_t i = 0; i < images.size(); ++i) {
		const std::string left_image(data_dir + "\\cam0\\data\\" + images[i]);
		const std::string right_image(data_dir + "\\cam1\\data\\" + images[i]);
		cv::Mat left = cv::imread(left_image);
		cv::Mat right = cv::imread(right_image);
		//  cv::Mat img2 = cv::imread(file_name2);
		if (left.empty() || right.empty()) {
			std::cout << "error" << std::endl;
			return;
		}
		visg.Run(left, right);
	}
}
