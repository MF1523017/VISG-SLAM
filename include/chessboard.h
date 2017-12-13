#ifndef CHESSBOARD_H
#define CHESSBOARD_H
#include "common.h"
namespace VISG {
	class Chessboard {
	public:
		using Ptr = std::shared_ptr<Chessboard>;
		Chessboard() = default;
		Chessboard(size_t row, size_t col,float grid_length);
		static Chessboard::Ptr handle(size_t row, size_t col, float grid_length) {
			static Chessboard::Ptr chess(new Chessboard(row, col, grid_length));
			return chess;
		}
		bool GetPose(cv::Mat &img,const cv::Mat &camera_matrix,Eigen::Matrix3f &R,Eigen::Vector3f &t,bool is_showing = true);
	private:

		bool is_first_frame_;
		cv::Size pattern_size_;
		float grid_length_;
		std::vector<cv::Point3f> points3_;
		Eigen::Matrix4f T0_;
		Eigen::Matrix3f R0_;
		Eigen::Vector3f t0_;
	};
}
#endif