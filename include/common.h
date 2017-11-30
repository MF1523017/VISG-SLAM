#ifndef COMMON_H
#define COMMON_H
/*
lMr: the right image corr to left image corr
*/
#include <opencv2/opencv.hpp>
#include <memory>
namespace VISG {
	struct Common {
		static size_t Height;
		static size_t Weight;
		static float Fx, Fy, Cx, Cy;
		static cv::Mat lRr;
		static cv::Mat ltr;
		static size_t HistBin;
	};

}
#endif // ! COMMON_h

