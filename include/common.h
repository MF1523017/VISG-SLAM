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

#include "type.h"


#define USE_PROJECT_ERROR
#define DRAW
namespace VISG {
	struct Common {
		static size_t Height;
		static size_t Width;
		static float Fx, Fy, Cx, Cy;
		static float FxInv, FyInv;
		static cv::Mat K;
		static cv::Mat lRr;
		static cv::Mat ltr;
		static float BaseLine;
		static size_t HistBin;
		static int BestOrbDistance;
		static size_t FeaturesNum;
		static float ScaleFactor;
		static size_t Levels;
	};

}
#endif // ! COMMON_h

