#include "common.h"
namespace VISG {
	size_t Common::Height = 0;
	size_t Common::Width = 0;
	float Common::Fx = .0;
	float Common::Fy = .0;
	float Common::Cx = .0;
	float Common::Cy = .0;
	float Common::FxInv = .0;
	float Common::FyInv = .0;
	cv::Mat Common::K = cv::Mat::eye(3, 3, CV_32F);
	cv::Mat Common::lRr = cv::Mat::eye(3, 3, CV_32F);
	cv::Mat Common::ltr = cv::Mat::zeros(3, 1, CV_32F);
	cv::Mat Common::DistCoeffs = cv::Mat::zeros(4, 1, CV_32F);
	float Common::BaseLine = 0.12;
	size_t Common::HistBin = 200;
	int Common::BestOrbDistance = 80;
	size_t Common::FeaturesNum = 2000;
	size_t Common::KeyFrameTh = Common::FeaturesNum / 10;
	float Common::ScaleFactor = 1.2;
	size_t Common::Levels = 8;
	size_t Common::EveryNFrames = 2;
	double Common::LoopClosingTh = 0.045;
}