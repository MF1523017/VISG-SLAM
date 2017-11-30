#ifndef MATCHER_H
#define MATCHER_H
#include "common.h"
namespace VISG {
	class Matcher {
	public:
		static int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);
	};
}

#endif //matcher.h