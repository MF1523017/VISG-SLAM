#include "frame.h"
#include <thread>

namespace VISG {
void Frame::ExtractFeature(const cv::Mat&left, const cv::Mat&right) {
	std::thread thread_left(&Frame::ExtractORB, this, left, true);
	std::thread thread_right(&Frame::ExtractORB, this, right, false);
	thread_left.join();
	thread_right.join();
	left_descriptors = p_orb_left_->descriptors_;
	right_descriptors = p_orb_right_->descriptors_;
	left_key_points = p_orb_left_->key_points_;
	right_key_points = p_orb_right_->key_points_;
}
void Frame::ExtractORB(const cv::Mat&img, bool is_left) {
	if (is_left)
		p_orb_left_->Extract(img);
	else
		p_orb_right_->Extract(img);
}
}