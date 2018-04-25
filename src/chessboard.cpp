#include "chessboard.h"
#include "utils.hpp"

namespace VISG {
	Chessboard::Chessboard(size_t row, size_t col, float grid_length):is_first_frame_(true),grid_length_(grid_length) {
		pattern_size_.height = row;
		pattern_size_.width = col;
		points3_.reserve(row*col);
		for (size_t i = 0; i < row; ++i) {
			for (size_t j = 0; j < col; ++j) {
				points3_.push_back(cv::Point3f(grid_length*j, grid_length*i, 0));
				std::cout << "[Chessboard::Chessboard] points3_: "<< points3_.back() << std::endl;
			}
		}
		T0_.setIdentity();
		R0_.setIdentity();
		t0_.setZero();
	}

	bool Chessboard::GetPose(cv::Mat &img, const cv::Mat &camera_matrix, Eigen::Matrix3f &R, Eigen::Vector3f &t,bool is_showing) {
		cv::Mat gray;
		if(img.channels() == 3)
			cv::cvtColor(img, gray, CV_RGB2GRAY);
		std::vector<cv::Point2f> corners;
		

		bool patternfound = cv::findChessboardCorners(gray, pattern_size_, corners,
			cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);
		
		if (corners.empty()){
			std::cout << "[Chessboard::GetPose] corners is empty" << std::endl;
			return false;
		}
		if (patternfound)
			cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
				cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
		cv::Mat Rvec(3,1,CV_32F),tvec(3,1,CV_32F),cvR(3,3,CV_32F),mask;
		bool ret = true;
		//R t: brings points from the model coordinate system to the camera coordinate system
		 ret = cv::solvePnPRansac(points3_, corners, camera_matrix, cv::Mat(), Rvec, tvec, true, 100, 8, 0.99, mask);
		if (!ret) {
			std::cout << "[Chessboard::GetPose] pnp error" << std::endl;
			return false;
		}

		std::vector<cv::Point2f> pro_points;
		cv::projectPoints(points3_, Rvec, tvec, Common::K, cv::Mat(), pro_points);
		float pro_error = 0;
		for (size_t i = 0; i < corners.size(); ++i) {
			cv::Point2f error = corners[i] - pro_points[i];
			pro_error += (fabs(error.x) + fabs(error.y));
		}
		std::cout << "[Chessboard::GetPose] pro_error: " << pro_error / corners.size() << std::endl;
		int valid_count = cv::countNonZero(mask);
		if (valid_count < 10 || static_cast<double>(valid_count) / corners.size() < 0.6)
			return false;

	//	PnpSolver solver;
//		solver.Solve(corners, points3_, Rvec, tvec);
		cv::Rodrigues(Rvec, cvR);
		Eigen::Matrix3f tmp_R = Rcv2Eigen(cvR);
		Eigen::Vector3f tmp_t = Tcv2Eigen(tvec);
		
		if (is_first_frame_) {
			R0_ = tmp_R;
			t0_ = tmp_t;
			T0_ = HPose(R0_, t0_);
			is_first_frame_ = false;
			std::cout << "[Chessboard::GetPose] T0_: " << T0_ << std::endl;
		}
		
		Eigen::Matrix4f ciTw = HPose(tmp_R, tmp_t);
		// TODO make sure the transform 
		Eigen::Matrix4f c0Tci =  T0_ * ciTw.inverse();
		HPose2Rt(c0Tci, R, t);
		if (is_showing) {
			cv::namedWindow("draw corners", 0);
			Eigen::Vector3f ypr(R2ypr(R));
			char text_rotation[128];
			char text_translation[128];
			snprintf(text_rotation, 128, "rotation: %3.3f, %3.3f, %3.3f", ypr.x(), ypr.y(), ypr.z());
			snprintf(text_translation, 128, "translation: %3.3f, %3.3f, %3.3f", t.x(), t.y(), t.z());
			cv::putText(gray, text_rotation, cv::Point2i(100, 20), 0, 0.5, cv::Scalar(255, 0, 255));
			cv::putText(gray, text_translation, cv::Point2i(100, 50), 0, 0.5, cv::Scalar(255, 0, 255));
			cv::drawChessboardCorners(img, pattern_size_, corners, patternfound);
			cv::imshow("draw corners", img);
		}
		return true;
	}

}