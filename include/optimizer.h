#ifndef OPTIMIZER_H
#define OPTIMIZER_H
#include "common.h"
namespace VISG {
	class Optimizer {
	public:
		static float ProjectPoints(const std::vector<cv::Point3f> &map_points,
			const std::vector<cv::Point2f> &img_points,
			const Eigen::Matrix3f &R, const Eigen::Vector3f &t, 
			std::vector<cv::Point2f> &pro_points);
		static float ProjectPointsStereoMatch(const MapPoints& map_points,
			const MatchPoints &match_points,
			const std::vector<bool> &inliers,
			const Eigen::Matrix3f &R, const Eigen::Vector3f &t,
			std::vector<cv::Point2f> &pro_points);
		static float ProjectPointsRefTrack2D3D(const MapPoints& map_points, 
			const KeyPoints & key_points,
			const MyMatches &matches,
			const Eigen::Matrix3f &R, const Eigen::Vector3f &t,
			std::vector<cv::Point2f> &pro_points);
};
}
#endif