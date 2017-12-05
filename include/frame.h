#ifndef FRAME_H
#define FRAME_H
#include "feature.h"
#include <vector>
namespace VISG {
class Frame {
public:
	using Ptr = std::shared_ptr<Frame>;
	Frame():p_orb_left_(new OrbFeature), p_orb_right_(new OrbFeature) {
		wRc.setIdentity();
		wTc.setZero();
	}
	void SetPose(const Eigen::Matrix3f &R, const Eigen::Vector3f &t);
	void ExtractFeatures(const cv::Mat&left, const cv::Mat&right);
	void ExtractORB(const cv::Mat&img, bool is_left);
	void Match(KeyPoints &left_key_points,const cv::Mat &left_descriptors,KeyPoints &right_key_points,const cv::Mat&right_descriptors);
	void StereoMatch();
	bool RefTrack(Frame::Ptr p_frame_ref, MyMatches&inliers_matches);
	bool IsKeyFrame(KeyPoints &key_points);
public:
	
	cv::Mat left_descriptors, right_descriptors;
	KeyPoints left_key_points, right_key_points;
	MatchPoints match_points;// (ul,vl,ur)
	MapPoints map_points;
	//cv::Mat wRc;//camera to world;
	//cv::Mat wTc;
	Eigen::Matrix3f wRc;
	Eigen::Vector3f wTc;
	std::vector<bool> inliers;
private:
	
	void Hist(KeyPoints &key_points, std::vector<std::vector<size_t>> &hist);
	void Reset();
	bool RecoverPose(const std::vector<cv::Point2f> &points1, const std::vector<cv::Point2f> &points2, const MyMatches &matches,MyMatches &inliers_matches);
	Feature::Ptr p_orb_left_;
	Feature::Ptr p_orb_right_;

};

}

#endif //frame.h
