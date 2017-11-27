#include "test.h"
#include "camera.h"
using namespace VISG;
void test_Camera(){
	// Create a ZED camera object
	Camera zed;

	// Open the camera
	if (!zed.Open(2,60)) {
		std::cout << "zed open error." << std::endl;
		return;
	}

	zed.SetCameraSettings(4, 4, 0, 5, 80, 80, 3500);

	// Capture 50 frames and stop
	int i = 0;
	cv::namedWindow("image", 0);
	cv::Mat left, right;
	while (i < 50) {
		// Grab an image
		if (zed.Grab(left, right)) {
			cv::imshow("image", left);
			std::cout << "[zed] timestamp: " << zed.GetTimestamp() << " fps: " << zed.GetFPS() << std::endl;
			int key = cv::waitKey(5);
			if (key == 27)
				break;
		}
	}
	zed.Close();
}