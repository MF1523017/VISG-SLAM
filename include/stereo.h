#ifndef STEREO_H
#define STEREO_H
#include "common.h"
namespace VISG {
	class Stereo {
	public:
		Stereo() {
			//p_sm = cv::StereoBM::create(16, 9);
			p_sm = cv::StereoSGBM::create(0,16,3);
		}
		void Compute(cv::Mat &left, cv::Mat &right, cv::Mat &disp,cv::Mat &points3);
	private:
		cv::Ptr<cv::StereoSGBM> p_sm;
	};
}

#endif // stereo.h