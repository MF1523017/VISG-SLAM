#ifndef KEYFRAME_H
#define KEYFRAME_H
#include "common.h"
#include "frame.h"
namespace VISG {
	class KeyFrame {
	public:
		KeyFrame() = default;
		KeyFrame(Frame::Ptr p_frame, cv::Mat &left, cv::Mat &right) {
			left_.copyTo(left_);
			right_.copyTo(right_);
			wRc = p_frame->wRc;
			wTc = p_frame->wTc;
		}
		void Match();
	public:
		using Ptr = std::shared_ptr<KeyFrame>;
		static size_t keys_th_tracked, keys_th_created;//key points' threshold 
		Eigen::Matrix3f wRc;//camera to world;
		Eigen::Vector3f wTc;
	private:
		cv::Mat left_;
		cv::Mat right_;
		
	};
}

#endif //keyframe.h