#include "chessboard.h"
#include "utils.hpp"
#include "draw_board.h"
#include "optimizer.h"
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

	bool Chessboard::GetPose(cv::Mat &img, const cv::Mat &camera_matrix, Eigen::Matrix3f &R, Eigen::Vector3f &t) {
		cv::Mat gray;
		if(img.channels() == 3)
			cv::cvtColor(img, gray, CV_RGB2GRAY);
		std::vector<cv::Point2f> corners;
		
		bool patternfound = cv::findChessboardCorners(gray, pattern_size_, corners,
			cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);
		if (patternfound)
			cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
				cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
		/*cv::namedWindow("chessboard corners", 0);
		cv::drawChessboardCorners(gray, pattern_size_, corners, patternfound);
		cv::imshow("chessboard corners", gray);
		cv::waitKey();*/
		cv::Mat Rvec,tvec,cvR,mask;
		//R t: brings points from the model coordinate system to the camera coordinate system
		bool ret = cv::solvePnPRansac(points3_, corners, camera_matrix, cv::Mat(), Rvec, tvec, true, 100, 8, 0.99, mask);
		if (!ret) {
			std::cout << "[Chessboard::GetPose] pnp error" << std::endl;
			return false;
		}
		int valid_count = cv::countNonZero(mask);
		if (valid_count < 10 || static_cast<double>(valid_count) / corners.size() < 0.6)
			return false;

	
		cv::Rodrigues(Rvec, cvR);
		Eigen::Matrix3f tmp_R = Rcv2Eigen(cvR);
		Eigen::Vector3f tmp_t = Tcv2Eigen(tvec);
		PnpSolver solver;
		solver.Solve(corners, points3_, Rvec, tvec);
		if (is_first_frame_) {
			R0_ = tmp_R;
			t0_ = tmp_t;
			T0_ = HPose(R0_, t0_);
			is_first_frame_ = false;
			std::cout << "[Chessboard::GetPose] T0_: " << T0_ << std::endl;
		}
#ifdef USE_PROJECT_ERROR
		std::vector<cv::Point2f> pro_points;
		float e = Optimizer::ProjectPoints(points3_, corners, tmp_R.transpose(), -tmp_R.transpose()*tmp_t, pro_points);
		std::cout << "[Chessboard::GetPose] project error: " << e << std::endl;
#endif
		Eigen::Matrix4f ciTw = HPose(tmp_R, tmp_t);
		// TODO make sure the transform 
		Eigen::Matrix4f c0Tci =  T0_ * ciTw.inverse();
		//std::cout << "[Chessboard::GetPose] c0Tci: " << c0Tci << std::endl;
		HPose2Rt(c0Tci, R, t);

		DrawBoard::handle().DrawPose(img, R, t, ret);
		DrawBoard::handle().DrawCorner(img, pattern_size_, corners, patternfound);
		return true;
	}
}