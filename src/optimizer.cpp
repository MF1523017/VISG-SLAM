#include "optimizer.h"

namespace VISG {
	//R,t:brings points from the  camera coordinate system  to the world coordinate system
	// map_points : is in thi world coordinate system
	float Optimizer::ProjectPoints(const std::vector<cv::Point3f> &map_points, const std::vector<cv::Point2f> &img_points, const Eigen::Matrix3f &R, const Eigen::Vector3f &t,
		std::vector<cv::Point2f> &pro_points) {
		if (map_points.empty())
			return -1;
		pro_points.clear();
		pro_points.reserve(map_points.size());
		const float fx = Common::Fx;
		const float fy = Common::Fy;
		const float cx = Common::Cx;
		const float cy = Common::Cy;
		float project_error = 0;
		for (size_t i = 0; i < map_points.size(); ++i) {
			Eigen::Vector3f pw(map_points[i].x, map_points[i].y, map_points[i].z);
			Eigen::Vector3f pc = R.transpose() * (pw -t);
			float u = fx * pc.x() / pc.z() + cx;
			float v = fy * pc.y() / pc.z() + cy;
			const cv::Point2f pro_point = cv::Point2f(u, v);
			const cv::Point2f error = pro_point - img_points[i];
			project_error += fabs(error.x) + fabs(error.y);
			pro_points.push_back(pro_point);
		}
		return project_error / map_points.size();
	}

	float Optimizer::ProjectPointsStereoMatch(const MapPoints& map_points,
		const MatchPoints &match_points,
		const std::vector<bool> &inliers,
		const Eigen::Matrix3f &R, const Eigen::Vector3f &t,
		std::vector<cv::Point2f> &pro_points) {
		if (map_points.empty())
			return -1;
		pro_points.clear();
		pro_points.reserve(map_points.size());
		const float fx = Common::Fx;
		const float fy = Common::Fy;
		const float cx = Common::Cx;
		const float cy = Common::Cy;
		float project_error = 0;
		int counter = 0;
		for (size_t i = 0; i < map_points.size(); ++i) {
			if (!inliers[i])
				continue;
			
			Eigen::Vector3f pc = R.transpose() * (map_points[i] - t);
			float lu = fx * pc.x() / pc.z() + cx;
			float lv = fy * pc.y() / pc.z() + cy;
			float ru = fx * (pc.x() - Common::BaseLine) / pc.z() + cx;
			const cv::Point2f pro_point = cv::Point2f(lu, lv);
			project_error += fabs(lu - match_points[i].x()) + fabs(lv - match_points[i].y()) + fabs(ru - match_points[i].z());
			pro_points.push_back(pro_point);
			++counter;
		}
		return project_error / counter;
	}

	float Optimizer::ProjectPointsRefTrack2D3D(const MapPoints& map_points, 
		const KeyPoints & key_points,
		const MyMatches &matches,
		const Eigen::Matrix3f &R, const Eigen::Vector3f &t,
		std::vector<cv::Point2f> &pro_points) {

		if (map_points.empty())
			return -1;
		pro_points.clear();
		pro_points.reserve(map_points.size());
		const float fx = Common::Fx;
		const float fy = Common::Fy;
		const float cx = Common::Cx;
		const float cy = Common::Cy;
		float project_error = 0;
		for (size_t i = 0; i < matches.size(); ++i) {
			const size_t queryIdx = matches[i].first;
			const size_t trainIdx = matches[i].second;
			Eigen::Vector3f pc = R.transpose() * (map_points[queryIdx] - t);
			if (pc.z() <= 0)
				continue;
			float u = fx * pc.x() / pc.z() + cx;
			float v = fy * pc.y() / pc.z() + cy;
			const cv::Point2f pro_point = cv::Point2f(u, v);
			const cv::Point2f error = pro_point - key_points[trainIdx].pt;
			project_error += fabs(error.x) + fabs(error.y);
			pro_points.push_back(pro_point);
		}
		return project_error / matches.size();
	}
}