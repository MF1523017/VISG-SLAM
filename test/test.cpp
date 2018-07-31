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
#include "loop.h"
#include <thread>

#define SAVE_POINTS

#define LOOP_CLOSING

//#define TRAINING

std::vector<float> errors;

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

	std::vector<std::string> images;

	//720 mode
	//const std::string data_dir("H:\\dataset\\20171207_demo\\20171207");
	//const std::string data_dir("H:\\dataset\\20171120\\20171120");

	// vga mode
	//const std::string data_dir("H:\\dataset\\20171214_1\\20171214");
	//const std::string data_dir("H:\\dataset\\20180423_0");
	//const std::string data_dir("H:\\dataset\\20180423_1");
	const std::string data_dir("H:\\dataset\\20180424_0");
	//const std::string data_dir("H:\\dataset\\20180502_0");
	
	//chessboard
	//const std::string data_dir("H:\\dataset\\20171207_chessboard\\20171207");

#ifdef LOOP_CLOSING
	//dictionary
	const std::string dict("H:\\my_only\\code\\slam\\ORB-SLAM2\\ORB_SLAM2\\Vocabulary\\ORBvoc.txt\\ORBvoc.txt");
	VisgSlamOffline visg(dict);
#else
	VisgSlamOffline visg;
#endif

	loadImage(data_dir, images);
	Timer timer;
	for (size_t i = 0; i < images.size(); ++i) {
		timer.Reset();
		const std::string left_image(data_dir + "\\cam0\\data\\" + images[i]);
		const std::string right_image(data_dir + "\\cam1\\data\\" + images[i]);
		cv::Mat left = cv::imread(left_image);
		cv::Mat right = cv::imread(right_image);
		if (left.empty() || right.empty()) {
			std::cout << "[test_offline] image empty error" << std::endl;
			return;
		}
		visg.Run(left, right);
#ifdef SAVE_POINTS
		const size_t pos = images[i].find('.');
		const std::string points_file(data_dir + "\\obj\\" + images[i].substr(0, pos) + ".obj");
		visg.SaveMapPoints(points_file);
#endif
		std::cout << "[test_offline] Time elapsed(ms): " << timer.ElapsedMS() << std::endl;
	}

	// save rt
	SaveT(data_dir + "\\position_groundtruth.txt", visg.positions_groundtruth());
	SaveT(data_dir + "\\position_slam.txt", visg.positions_slam());
	SaveT(data_dir + "\\position_key_frame.txt", visg.positions_key_frame());
	 // save errors
	std::ofstream of(data_dir + "\\errors.txt");
	for (const auto & e: errors) {
		of << e << std::endl;
	}
	of.close();
}

void test_stereo(){
	VisgSlamOffline visg;
	Stereo stereo;
	std::string points_file("H:\\dataset\\20180423_0\\stereo\\points.obj");
	std::string left_file("H:\\dataset\\20180423_0\\cam0\\data\\2492355038416.jpg");
	std::string right_file("H:\\dataset\\20180423_0\\cam1\\data\\2492355038416.jpg");
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

void test_loop() {
	std::vector<std::string> db_images,db_image_names,test_images,test_image_names;
	// vga mode
	const std::string db_dir("H:\\dataset\\20171214_1\\20171214");
	const std::string test_dir("H:\\dataset\\20171214_1\\loop");
	const std::string dictionary("H:\\dataset\\20171214_1\\20171214\\vocabulary.yml.gz");
	Loop loop;
#ifdef TRAINING
	loadImage(db_dir, db_images);
	db_image_names.reserve(db_images.size());
	for (size_t i = 0; i < db_images.size(); ++i) {
		const std::string left_image(db_dir + "\\cam0\\data\\" + db_images[i]);
		db_image_names.push_back(left_image);
	}
	loop.MakeDictionary(db_image_names);
	loop.SaveDictionary(dictionary);
#else
	const std::string dictionary_file("H:\\my_only\\code\\slam\\ORB-SLAM2\\ORB_SLAM2\\Vocabulary\\ORBvoc.txt\\ORBvoc.txt");
	loop.LoadDictionary(dictionary_file);
#endif 
	loadImage(test_dir, test_images);
	test_image_names.reserve(test_images.size());
	for (size_t i = 0; i < test_images.size(); ++i) {
		const std::string left_image(test_dir + "\\cam0\\data\\" + test_images[i]);
		test_image_names.push_back(left_image);
	}

	std::vector<cv::Mat> descriptors;
	loop.LoadFeature(test_image_names, descriptors);
	loop.AddFeatureToDB(descriptors);
	DBoW3::QueryResults ret;
	for (int i = 0; i < descriptors.size(); ++i) {
		std::cout << "image " << i << " ";
		loop.ComputeSimilar(descriptors[i], ret);
	}
}

void test_EuRoCDataset() {
	// camera parameters
	Common::Height = 480;
	Common::Width = 752;
	Common::Fx = 458.654;
	Common::Fy = 457.296;
	Common::Cx = 367.215;
	Common::Cy = 248.375;
	Common::K = (cv::Mat_<float>(3, 3) << Common::Fx, 0, Common::Cx,
		0, Common::Fy, Common::Cy,
		0, 0, 1);
	Common::ltr.at<float>(0, 0) = 0.4790639384423901;
	Common::FxInv = 1.0 / Common::Fx;
	Common::FyInv = 1.0 / Common::Fy;
	std::cout << "[VisgSlamOffline]   K: " << Common::K << " R: "
		<< Common::lRr << " t: " << Common::ltr << std::endl;
	Common::BaseLine = 0.4790639384423901;
	Common::DistCoeffs.at<float>(0) = -0.28340811;
	Common::DistCoeffs.at<float>(1) = 0.07395907;
	Common::DistCoeffs.at<float>(2) = 0.00019359;
	Common::DistCoeffs.at<float>(3) = 1.76187114e-05;

	const std::string data_dir("H:\\dataset\\MH_01_easy\\mav0");
	std::vector<std::string> images;

#ifdef LOOP_CLOSING
	//dictionary
	const std::string dict("H:\\my_only\\code\\slam\\ORB-SLAM2\\ORB_SLAM2\\Vocabulary\\ORBvoc.txt\\ORBvoc.txt");
	VisgSlamOffline visg(dict);
#else
	VisgSlamOffline visg;
#endif

	loadImage(data_dir, images);
	Timer timer;
	for (size_t i = 0; i < images.size(); ++i) {
		timer.Reset();
		const std::string left_image(data_dir + "\\cam0\\data\\" + images[i]);
		const std::string right_image(data_dir + "\\cam1\\data\\" + images[i]);
		cv::Mat left = cv::imread(left_image);
		cv::Mat right = cv::imread(right_image);
		if (left.empty() || right.empty()) {
			std::cout << "[test_offline] image empty error" << std::endl;
			return;
		}
		visg.Run(left, right);
#ifdef SAVE_POINTS
		const size_t pos = images[i].find('.');
		const std::string points_file(data_dir + "\\obj\\" + images[i].substr(0, pos) + ".obj");
		visg.SaveMapPoints(points_file);
#endif
		std::cout << "[test_offline] Time elapsed(ms): " << timer.ElapsedMS() << std::endl;
	}

	// save rt
	SaveT(data_dir + "\\position_groundtruth.txt", visg.positions_groundtruth());
	SaveT(data_dir + "\\position_slam.txt", visg.positions_slam());
	SaveT(data_dir + "\\position_key_frame.txt", visg.positions_key_frame());
	// save errors
	std::ofstream of(data_dir + "\\errors.txt");
	for (const auto & e : errors) {
		of << e << std::endl;
	}
	of.close();

}



void test_KITTIDataset() {
	// camera parameters
	Common::Height = 5.120000e+02;
	Common::Width = 1.392000e+03;
	Common::Fx = 9.842439e+02;
	Common::Fy = 9.808141e+02;
	Common::Cx = 6.900000e+02;
	Common::Cy = 2.331966e+02;
	Common::K = (cv::Mat_<float>(3, 3) << Common::Fx, 0, Common::Cx,
		0, Common::Fy, Common::Cy,
		0, 0, 1);
	Common::ltr.at<float>(0, 0) = 5.370000e-01;
	Common::FxInv = 1.0 / Common::Fx;
	Common::FyInv = 1.0 / Common::Fy;
	std::cout << "[VisgSlamOffline]   K: " << Common::K << " R: "
		<< Common::lRr << " t: " << Common::ltr << std::endl;
	Common::BaseLine = 5.370000e-01;

	Common::lRr = (cv::Mat_<float>(3, 3) << 9.993513e-01, 1.860866e-02, -3.083487e-02,
		-1.887662e-02, 9.997863e-01, -8.421873e-03,
		3.067156e-02, 8.998467e-03, 9.994890e-01);

	Common::DistCoeffs.at<float>(0) = -0.28340811;
	Common::DistCoeffs.at<float>(1) = 0.07395907;
	Common::DistCoeffs.at<float>(2) = 0.00019359;
	Common::DistCoeffs.at<float>(3) = 1.76187114e-05;

	const std::string data_dir("H:\\my_only\\dataset\\lidar_net_dataset\\2011_09_26_drive_0093_extract\\2011_09_26\\2011_09_26_drive_0093_extract\\");
	std::vector<std::string> images;

#ifdef LOOP_CLOSING
	//dictionary
	const std::string dict("H:\\my_only\\code\\slam\\ORB-SLAM2\\ORB_SLAM2\\Vocabulary\\ORBvoc.txt\\ORBvoc.txt");
	VisgSlamOffline visg(dict);
#else
	VisgSlamOffline visg;
#endif

	loadImage(data_dir, images);
	Timer timer;
	for (size_t i = 0; i < images.size(); ++i) {
		timer.Reset();
		const std::string left_image(data_dir + "\\cam0\\data\\" + images[i]);
		const std::string right_image(data_dir + "\\cam1\\data\\" + images[i]);
		cv::Mat left = cv::imread(left_image);
		cv::Mat right = cv::imread(right_image);
		if (left.empty() || right.empty()) {
			std::cout << "[test_offline] image empty error" << std::endl;
			return;
		}
		visg.Run(left, right);
#ifdef SAVE_POINTS
		const size_t pos = images[i].find('.');
		const std::string points_file(data_dir + "\\obj\\" + images[i].substr(0, pos) + ".obj");
		visg.SaveMapPoints(points_file);
#endif
		std::cout << "[test_offline] Time elapsed(ms): " << timer.ElapsedMS() << std::endl;
	}

	// save rt
	SaveT(data_dir + "\\position_groundtruth.txt", visg.positions_groundtruth());
	SaveT(data_dir + "\\position_slam.txt", visg.positions_slam());
	SaveT(data_dir + "\\position_key_frame.txt", visg.positions_key_frame());
	// save errors
	std::ofstream of(data_dir + "\\errors.txt");
	for (const auto & e : errors) {
		of << e << std::endl;
	}
	of.close();

}
