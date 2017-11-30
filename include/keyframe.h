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
			p_frame->wRc.copyTo(wRc);
			p_frame->wTc.copyTo(wTc);
		}
	
		void Match();
	public:
		using Ptr = std::shared_ptr<KeyFrame>;
		static size_t keys_th_tracked, keys_th_created;//key points' threshold 
		cv::Mat wRc;//camera to world;
		cv::Mat wTc;
	private:
		cv::Mat left_;
		cv::Mat right_;
		
	};
}

#endif //keyframe.h