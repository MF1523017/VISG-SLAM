#include "visg_slam.h"
#include "draw_board.h"
//#define USE_CHESSBOARD
//#define HDMODE
namespace VISG {
	VisgSlam::VisgSlam():tracker_(new OrbTracker) {
		zed_.Open(3,100);
		Common::Height = zed_.cam_info.left_cam.image_size.height;
		Common::Width = zed_.cam_info.left_cam.image_size.width;
		Common::Fx = zed_.cam_info.left_cam.fx;
		Common::Fy = zed_.cam_info.left_cam.fy;
		Common::Cx = zed_.cam_info.left_cam.cx;
		Common::Cy = zed_.cam_info.left_cam.cy;
		Common::K = (cv::Mat_<float>(3, 3) << Common::Fx, 0, Common::Cx,
			0, Common::Fy, Common::Cy,
			0, 0, 1);

		Common::lRr = zed_.cam_info.extrinsic.R.clone();
		Common::ltr = zed_.cam_info.extrinsic.t.clone();
		Common::BaseLine = Common::ltr.at<float>(0, 0);
		Common::FxInv = 1.0 / Common::Fx;
		Common::FyInv = 1.0 / Common::Fy;
		//std::cout << zed_.cam_info << std::endl;
	}
	void VisgSlam::Run(cv::Mat left, cv::Mat right) {
		while (true) {
			// Grab an image
			if (zed_.Grab(left_, right_)) {
				//cv::imwrite("1.png", left_);
				/*cv::cvtColor(left_, left, CV_RGBA2GRAY);
				cv::cvtColor(right_, right, CV_RGBA2GRAY);
				(*tracker_)(left, right);*/
				(*tracker_)(left_, right_);
				std::cout << "fps: " << zed_.GetFPS() << std::endl;
				if (cv::waitKey(10) == 27)
					break;
			}
		}
	}
	void VisgSlam::RecordImages(const std::string &data_dir) {
		const std::string left_dir(data_dir + "\\cam0\\");
		const std::string right_dir(data_dir + "\\cam1\\");
		cv::namedWindow("left", 0);
		std::cout << "please press 's' to save image!" << std::endl;
		bool IsSave = false;
		while (true) {
			// Grab an image
			if (zed_.Grab(left_, right_)) {
				//cv::imwrite("1.png", left_);
				//std::cout << "fps: " << zed_.GetFPS() << std::endl;
				auto timestamp = zed_.GetTimestamp();
				if (IsSave) {
					cv::imwrite(left_dir + std::to_string(timestamp) + ".jpg", left_);
					cv::imwrite(right_dir + std::to_string(timestamp) + ".jpg", right_);
				}
				cv::imshow("left", left_);
				int key = cv::waitKey(10);
				if(27 == key)
					break;
				else if ('s' == key) {
					IsSave = !IsSave;
					if (!IsSave) {
						std::cout << "images have been saved " << data_dir << std::endl;
						std::cout << "please press 's' to save image!" << std::endl;
					}
					else
						std::cout << "please press 's' to stop saving image!" << std::endl;
				}
			}
		}
		std::cout << "images have been saved " << data_dir << std::endl;
	}

	VisgSlamOffline::VisgSlamOffline() :tracker_(new OrbTracker) {

		// 720 mode
#ifdef HDMODE
		Common::Height = 720;
		Common::Width = 1280;
		Common::Fx = 695.822;
		Common::Fy = 695.822;
		Common::Cx = 638.049;
		Common::Cy = 371.577;
		Common::K = (cv::Mat_<float>(3, 3) << Common::Fx, 0, Common::Cx,
			0, Common::Fy, Common::Cy,
			0, 0, 1);
		Common::ltr.at<float>(0,0) = 0.12;
		Common::BaseLine = 0.12;
		Common::FxInv = 1.0 / Common::Fx;
		Common::FyInv = 1.0 / Common::Fy;
		std::cout << "[VisgSlamOffline] K: " << Common::K << " R: "
			<< Common::lRr << " t: " << Common::ltr << std::endl;
#else
		// vga mode
		
		Common::Height = 376;
		Common::Width = 672;
		Common::Fx = 345.766;
		Common::Fy = 345.766;
		Common::Cx = 334.065;
		Common::Cy = 193.254;
		Common::K = (cv::Mat_<float>(3, 3) << Common::Fx, 0, Common::Cx,
			0, Common::Fy, Common::Cy,
			0, 0, 1);
		Common::ltr.at<float>(0, 0) = 0.12;
		Common::FxInv = 1.0 / Common::Fx;
		Common::FyInv = 1.0 / Common::Fy;
		std::cout << "[VisgSlamOffline]   K: " << Common::K << " R: "
			<< Common::lRr << " t: " << Common::ltr << std::endl;
#endif
	}

	VisgSlamOffline::VisgSlamOffline(const std::string &dictionary) :tracker_(new OrbTracker(dictionary)) {

		// 720 mode
#ifdef HDMODE
		Common::Height = 720;
		Common::Width = 1280;
		Common::Fx = 695.822;
		Common::Fy = 695.822;
		Common::Cx = 638.049;
		Common::Cy = 371.577;
		Common::K = (cv::Mat_<float>(3, 3) << Common::Fx, 0, Common::Cx,
			0, Common::Fy, Common::Cy,
			0, 0, 1);
		Common::ltr.at<float>(0, 0) = 0.12;
		Common::BaseLine = 0.12;
		Common::FxInv = 1.0 / Common::Fx;
		Common::FyInv = 1.0 / Common::Fy;
		std::cout << "[VisgSlamOffline] K: " << Common::K << " R: "
			<< Common::lRr << " t: " << Common::ltr << std::endl;
#else
		// vga mode

		Common::Height = 376;
		Common::Width = 672;
		Common::Fx = 345.766;
		Common::Fy = 345.766;
		Common::Cx = 334.065;
		Common::Cy = 193.254;
		Common::K = (cv::Mat_<float>(3, 3) << Common::Fx, 0, Common::Cx,
			0, Common::Fy, Common::Cy,
			0, 0, 1);
		Common::ltr.at<float>(0, 0) = 0.12;
		Common::FxInv = 1.0 / Common::Fx;
		Common::FyInv = 1.0 / Common::Fy;
		std::cout << "[VisgSlamOffline]   K: " << Common::K << " R: "
			<< Common::lRr << " t: " << Common::ltr << std::endl;
#endif
	}

	void VisgSlamOffline::Run(cv::Mat &left, cv::Mat &right) {
		cv::Mat left_, right_;	
		Eigen::Matrix3f R_truth, R;
		Eigen::Vector3f t_truth, t;
#ifdef USE_CHESSBOARD	
		auto ret = Chessboard::handle(7, 6, 0.025)->GetPose(left, Common::K, R_truth, t_truth);
		//std::cout << "[VisgSlamOffline Run] chess get pose: " << ret << std::endl;
		if (!ret) {
			return;
		}
		(*tracker_)(left, right);
		tracker_->GetPose(R, t);
		std::cout << "[VisgSlamOffline Run] t error: " << (t_truth - t).transpose() << std::endl;
		if (t_truth.norm() > 10 || t.norm() > 10)
			return;
		positions_groundtruth_.push_back(t_truth);
		


#else
		(*tracker_)(left, right);
		tracker_->GetPose(R, t);
#endif
		positions_slam_.push_back(t);
		
		cv::waitKey(1);
	}

	void VisgSlamOffline::SaveMapPoints(const std::string &file_name) {
		std::vector<Eigen::Vector3f> map_points;
		std::vector<Eigen::Vector3i> colors;
		tracker_->GetMapPoints(map_points, colors);
		std::ofstream out_file(file_name);
		for (size_t i = 0; i < map_points.size(); ++i) {
			out_file << "v " << map_points[i].x() << " " << map_points[i].y() << " " << map_points[i].z() << " "
				<< colors[i].x() << " " << colors[i].y() << " " << colors[i].z() << std::endl;
		}
		out_file.close();
	}

	// get key_frame

	std::vector<Eigen::Vector3f> VisgSlamOffline::positions_key_frame() {
		tracker_->GetKeyFramePositions(positions_key_frame_);
		return positions_key_frame_;
	}
}