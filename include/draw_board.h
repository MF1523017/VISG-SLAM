#ifndef DRAW_BOARD_H
#define DRAW_BOARD_H
#include "type.h"
namespace VISG {
	struct Box {
		Eigen::Vector3f center;
		Eigen::Vector3f ori, cox, coy, coz;
		Eigen::Vector3f lix, liy, liz;
		float size;
		Eigen::Vector4f initPlane;

		Box(const Eigen::Vector3f &center_,float size_)
		{
			center = center_;
			size = size_;
		}
};
void OnMouse(int event, int x, int y, int flags, void*ustc);
class DrawBoard {
public:
	DrawBoard() :is_tap(false){
		cv::namedWindow("black_board", 0);
		cv::namedWindow("ar", 0);
		cv::setMouseCallback("ar", OnMouse);
	}
	static DrawBoard & handle() {
		static DrawBoard draw_board;
		return draw_board;
	}
	void DrawFeatures(cv::Mat &img, const KeyPoints &key_points,bool showing = true);
	void DrawMatch(cv::Mat &left, cv::Mat &right, MatchPoints &match_points);
	void DrawMatch(cv::Mat &left, cv::Mat &right, MyMatches &my_matches, const KeyPoints& key_points1, const KeyPoints& key_points2);
	void DrawPose(cv::Mat &left, const Eigen::Matrix3f & R, const Eigen::Vector3f &t,bool status);
	void DrawProjectError(cv::Mat &img, const KeyPoints &key_points,const MyMatches &matches,const std::vector<cv::Point2f> &pro_points);
	void DrawAR(cv::Mat &result,const std::vector<Eigen::Vector3f> &point_cloud, const Eigen::Matrix3f wRc,const Eigen::Vector3f &wtc);
	void ShowBlackBoard(cv::Mat &img) {
		cv::imshow("black_board", img);
	}
	void ShowAR(cv::Mat &img) {
		cv::imshow("ar", img);
	}
	void Clear() {
		is_tap = false;
		boxes_.clear();
	}

private:
	Eigen::Vector3f FindGround(const std::vector<Eigen::Vector3f> &point_cloud, std::vector<Eigen::Vector3f> &inlier_points);
	void DrawGround(cv::Mat &result, std::vector<Eigen::Vector3f> &point_cloud, const Eigen::Matrix3f wRc, const Eigen::Vector3f &wtc);
	void DrawBox(cv::Mat &result, const Box &box, const Eigen::Matrix3f wRc, const Eigen::Vector3f &wtc);
	Eigen::Vector4f FindPlane(std::vector<Eigen::Vector3f> &point_cloud);
	Eigen::Vector4f CreatPlane(const Eigen::Vector3f &p1, const Eigen::Vector3f &p2, const Eigen::Vector3f &p3);
public:
	float tap_x, tap_y;
	bool is_tap;
private:
	std::vector<Box> boxes_;
};
}
#endif
