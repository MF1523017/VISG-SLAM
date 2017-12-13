#ifndef FRAME_H
#define FRAME_H
#include "feature.h"
#include <vector>
namespace VISG {
class Frame {
public:
	using Ptr = std::shared_ptr<Frame>;
	Frame():p_orb_left_(new OrbFeature), p_orb_right_(new OrbFeature) {
		wRc << 1, 0, 0,
			0, 1, 0,
			0, 0, 1;
		wtc<< 0,0,0;
		wTc<<1,0,0,0,
			0,1,0,0,
			0,0,1,0,
			0,0,0,1;
		rRc = wRc;
		rtc = wtc;
	}
	void SetPose(const Eigen::Matrix3f &R, const Eigen::Vector3f &t);
	void ExtractFeatures(const cv::Mat&left, const cv::Mat&right);
	void ExtractORB(const cv::Mat&img, bool is_left);
	void Match(KeyPoints &left_key_points,const cv::Mat &left_descriptors,KeyPoints &right_key_points,const cv::Mat&right_descriptors);
	size_t StereoMatch();
	bool RefTrack2D2D(Frame::Ptr p_frame_ref, MyMatches&inliers_matches);
	bool RefTrack2D3D(Frame::Ptr p_frame_ref, MyMatches &inliers_matches);
	bool IsKeyFrame(MyMatches &matches);
	void GetwMapPoints(MapPoints &valid_map_points);
public:
	
	cv::Mat left_descriptors, right_descriptors;
	KeyPoints left_key_points, right_key_points;
	MatchPoints match_points;// (ul,vl,ur)
	MapPoints map_points;
	//cv::Mat wRc;//camera to world;
	//cv::Mat wTc;
	Eigen::Matrix3f wRc;//rotation camera to world frame
	Eigen::Vector3f wtc;//translation
	Eigen::Matrix3f rRc;// camera to reference frame
	Eigen::Vector3f rtc;
	Eigen::Matrix4f wTc;// pose
	std::vector<bool> inliers;
private:
	
	void Hist(KeyPoints &key_points, std::vector<std::vector<size_t>> &hist);
	void Reset();
	bool RecoverPose(const std::vector<cv::Point2f> &points1, const std::vector<cv::Point2f> &points2, const MyMatches &matches,
		MyMatches &inliers_matches,cv::Mat &R,cv::Mat &t);
	bool RecoverPose(const std::vector<cv::Point2f> &points2, const std::vector<cv::Point3f> &points3, const MyMatches &matches,
		MyMatches &inliers_matches, cv::Mat &R, cv::Mat &t);
	bool RecoverPoseWithPnpSolver(const std::vector<cv::Point2f> &points2, const std::vector<cv::Point3f> &points3, cv::Mat &R, cv::Mat &t);
	Feature::Ptr p_orb_left_;
	Feature::Ptr p_orb_right_;

};

}

#endif //frame.h
