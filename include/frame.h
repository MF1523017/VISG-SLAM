#ifndef FRAME_H
#define FRAME_H
#include "feature.h"
#include <vector>
namespace VISG {
class Frame {
public:
	Frame():p_orb_left_(new OrbFeature), p_orb_right_(new OrbFeature) {}
	void ExtractFeatures(const cv::Mat&left, const cv::Mat&right);
	void ExtractORB(const cv::Mat&img, bool is_left);
	void Match(KeyPoints &left_key_points,const cv::Mat &left_descriptors,KeyPoints &right_key_points,const cv::Mat&right_descriptors);
	void StereoMatch();
	bool IsKeyFrame(KeyPoints &key_points);
public:
	using Ptr = std::shared_ptr<Frame>;
	cv::Mat left_descriptors, right_descriptors;
	KeyPoints left_key_points, right_key_points;
	MatchPoints match_points;// (ul,vl,ur)
	cv::Mat wRc;//camera to world;
	cv::Mat wTc;
private:
	
	void Hist(KeyPoints &key_points, std::vector<std::vector<size_t>> &hist);
	
	Feature::Ptr p_orb_left_;
	Feature::Ptr p_orb_right_;

};

}

#endif //frame.h
