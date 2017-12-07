#include "draw_board.h"
#include "utils.hpp"

namespace VISG {
void DrawBoard::DrawFeatures(cv::Mat &img, const KeyPoints &key_points, bool showing){
	cv::Mat img_key_points;
	cv::drawKeypoints(img, key_points, img_key_points, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
	cv::imshow("black_board", img_key_points);
	if(showing)
		while (cv::waitKey() == 27);
}
void DrawBoard::DrawMatch(cv::Mat &left, cv::Mat &right, MatchPoints &match_points, bool showing) {
	cv::Size left_size = left.size();
	cv::Size right_size = right.size();

	cv::Size size(left_size.width + right_size.width, left_size.height);
	cv::Mat large(size, left.type());
	large.setTo(0);
	cv::Mat img1 = large(cv::Rect(0, 0, left_size.width, left_size.height));
	cv::Mat img2 = large(cv::Rect(left_size.width, 0, right_size.width, left_size.height));
	left.copyTo(img1);
	right.copyTo(img2);
	for (size_t i = 0; i < match_points.size(); ++i) {
		cv::Point2f lp(match_points[i].x(), match_points[i].y());
		cv::Point2f rp(match_points[i].z() + left_size.width, match_points[i].y());
		cv::circle(large, lp, 10, cv::Scalar(255, 0, 0));
		cv::circle(large, rp, 10, cv::Scalar(255, 0, 0));
		cv::line(large, lp, rp, cv::Scalar(0, 0, 255));
		cv::putText(large, std::to_string(i), lp, 0, 0.5, cv::Scalar(255, 0, 0));
		cv::putText(large, std::to_string(i), rp, 0, 0.5, cv::Scalar(255, 0, 0));
	}
	cv::imshow("black_board", large);
	if (showing)
		while (cv::waitKey() == 27);

}

void DrawBoard::DrawMatch(cv::Mat &left, cv::Mat &right, MyMatches &my_matches, const KeyPoints& key_points1, const KeyPoints& key_points2, bool showing) {
	cv::Size left_size = left.size();
	cv::Size right_size = right.size();

	cv::Size size(left_size.width + right_size.width, left_size.height);
	cv::Mat large(size, left.type());
	large.setTo(0);
	cv::Mat img1 = large(cv::Rect(0, 0, left_size.width, left_size.height));
	cv::Mat img2 = large(cv::Rect(left_size.width, 0, right_size.width, left_size.height));
	left.copyTo(img1);
	right.copyTo(img2);
	for (size_t i = 0; i < my_matches.size(); ++i) {
		const int & queryIdx = my_matches[i].first;
		const int & trainIdx = my_matches[i].second;
		cv::Point2f lp(key_points1[queryIdx].pt);
		cv::Point2f rp(key_points2[trainIdx].pt.x + left_size.width, key_points2[trainIdx].pt.y);
		cv::circle(large, lp, 10, cv::Scalar(255, 0, 0));
		cv::circle(large, rp, 10, cv::Scalar(255, 0, 0));
		cv::line(large, lp, rp, cv::Scalar(0, 0, 255));
		//cv::putText(large, std::to_string(i), lp, 0, 0.5, cv::Scalar(255, 0, 0));
		//cv::putText(large, std::to_string(i), rp, 0, 0.5, cv::Scalar(255, 0, 0));
	}
	cv::imshow("black_board", large);
	if (showing)
		while (cv::waitKey() == 27);

}
void DrawBoard::DrawPose(cv::Mat &left, const Eigen::Matrix3f & R, const Eigen::Vector3f &t,bool status) {
	Eigen::Vector3f ypr(R2ypr(R));
	char text_rotation[128];
	char text_translation[128];
	char test_status[128];
	snprintf(text_rotation, 128, "rotation: %3.2f, %3.2f, %3.2f", ypr.x(), ypr.y(), ypr.z());
	snprintf(text_translation, 128, "translation: %3.2f, %3.2f, %3.2f", t.x(), t.y(), t.z());
	snprintf(test_status, 128, "status: %d", status);
	cv::putText(left, text_rotation, cv::Point2i(100, 20), 0, 0.5, cv::Scalar(255, 0, 0));
	cv::putText(left, text_translation, cv::Point2i(100, 50), 0, 0.5, cv::Scalar(255, 0, 0));
	cv::putText(left, test_status, cv::Point2i(100, 80), 0, 0.5, cv::Scalar(255, 0, 0));
}

void DrawBoard::DrawCorner(cv::Mat &img,const cv::Size &pattern_size,const std::vector<cv::Point2f> & corners,bool patternfound) {
	cv::drawChessboardCorners(img, pattern_size, corners, patternfound);
	cv::imshow("black_board", img);
}

}