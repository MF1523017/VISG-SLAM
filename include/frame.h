#ifndef FRAME_H
#define FRAME_H
#include "feature.h"
namespace VISG {
class Frame {
public:
	Frame():p_orb_left_(new OrbFeature), p_orb_right_(new OrbFeature) {}
	void ExtractFeature(const cv::Mat&left, const cv::Mat&right);
	void ExtractORB(const cv::Mat&img, bool is_left);
public:
	using Ptr = std::shared_ptr<Frame>;
	cv::Mat left_descriptors, right_descriptors;
	KeyPoints left_key_points, right_key_points;
private:
	Feature::Ptr p_orb_left_;
	Feature::Ptr p_orb_right_;
};

}

#endif //frame.h
