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