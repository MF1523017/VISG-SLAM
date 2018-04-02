#include "test.h"
#include "feature.h"
#include "frame.h"
#include "draw_board.h"
#include "utils.hpp"
#include "camera.h"
#include "visg_slam.h"
#include "timer.h"
#include "stereo.h"
#include "thread_pool.h"
#include <thread>

#define SAVE_POINTS

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

	//720 mode
	//const std::string data_dir("H:\\dataset\\20171207_demo\\20171207");
	//const std::string data_dir("H:\\dataset\\20171120\\20171120");

	// vga mode
	const std::string data_dir("H:\\dataset\\20171214_1\\20171214");
	
	//chessboard
	//const std::string data_dir("H:\\dataset\\20171207_chessboard\\20171207");
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
#ifdef SAVE_POINTS
		const size_t pos = images[i].find('.');
		const std::string points_file(data_dir + "\\obj\\" + images[i].substr(0, pos) + ".obj");
		std::cout << "[test_offline] points_file: " << points_file << std::endl;
		visg.SaveMapPoints(points_file);
#endif
		std::cout << "[test_offline] Time elapsed(ms): " << timer.ElapsedMS() << std::endl;
	}
}

void test_stereo(){
	VisgSlamOffline visg;
	Stereo stereo;
	std::string points_file("H:\\dataset\\20171214_1\\20171214\\stereo\\points.obj");
	std::string left_file("H:\\dataset\\20171214_1\\20171214\\cam0\\data\\870747931678.jpg");
	std::string right_file("H:\\dataset\\20171214_1\\20171214\\cam1\\data\\870747931678.jpg");
	/*std::string left_file("H:\\opencv\\sources\\samples\\data\\aloeL.jpg");
	std::string right_file("H:\\opencv\\sources\\samples\\data\\aloeR.jpg");*/
	cv::Mat left = cv::imread(left_file,-1);
	cv::Mat right = cv::imread(right_file,-1);
	cv::Mat disp,points3;
	stereo.Compute(left, right, disp,points3);
	SaveXYZ(points_file.c_str(), points3,left);
}

void test_thread_pool() {
	std::mutex mtx;
	try
	{
		ThreadPool tp;
		std::vector<std::future<int>> v;
		std::vector<std::future<void>> v1;
		for (int i = 0; i <= 10; ++i)
		{
			auto ans = tp.Add([](int answer) { return answer; }, i);
			v.push_back(std::move(ans));
		}
		for (int i = 0; i <= 5; ++i)
		{
			auto ans = tp.Add([&mtx](const std::string& str1, const std::string& str2)
			{
				std::lock_guard<std::mutex> lg(mtx);
				std::cout << (str1 + str2) << std::endl;
				return;
			}, "hello ", "world");
			v1.push_back(std::move(ans));
		}
		for (size_t i = 0; i < v.size(); ++i)
		{
			std::lock_guard<std::mutex> lg(mtx);
			std::cout << v[i].get() << std::endl;
		}
		for (size_t i = 0; i < v1.size(); ++i)
		{
			v1[i].get();
		}
	}
	catch (std::exception& e)
	{
		std::cout << e.what() << std::endl;
	}

}
