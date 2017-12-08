#ifndef COMMON_H
#define COMMON_H
/*
lMr: the right image corr to left image corr
*/
#include <opencv2/opencv.hpp>
#include <memory>
#include <Eigen/Dense>
#include <utility>
#include <algorithm>
//#define USE_CHESSBOARD
namespace VISG {
	struct Common {
		static size_t Height;
		static size_t Weight;
		static float Fx, Fy, Cx, Cy;
		static float FxInv, FyInv;
		static cv::Mat K;
		static cv::Mat lRr;
		static cv::Mat ltr;
		static size_t HistBin;
		static int BestOrbDistance;
		static size_t FeaturesNum;
	};

}
#endif // ! COMMON_h

