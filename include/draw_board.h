#ifndef DRAW_BOARD_H
#define DRAW_BOARD_H
#include "type.h"
namespace VISG {
class DrawBoard {
public:
	DrawBoard() {
		cv::namedWindow("black_board", 0);
	}
	static DrawBoard & handle() {
		static DrawBoard draw_board;
		return draw_board;
	}
	void DrawFeatures(cv::Mat &img, const KeyPoints &key_points,bool showing = true);
	void DrawMatch(cv::Mat &left, cv::Mat &right, MatchPoints &match_points, bool showing = true);
	void DrawMatch(cv::Mat &left, cv::Mat &right, MyMatches &my_matches, const KeyPoints& key_points1, const KeyPoints& key_points2, bool showing = true);
	void DrawPose(cv::Mat &left, const Eigen::Matrix3f & R, const Eigen::Vector3f &t,bool status);
};
}
#endif
