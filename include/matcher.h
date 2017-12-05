#ifndef MATCHER_H
#define MATCHER_H
#include "common.h"
namespace VISG {
	class Matcher {
	public:
		static int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);
		static void OrbMatch(std::vector<std::pair<int, int>>& matches,const  cv::Mat &descriptors1, const  cv::Mat &descriptors2);
	};
}

#endif //matcher.h