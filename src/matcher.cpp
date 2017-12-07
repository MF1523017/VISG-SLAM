#include "matcher.h"
namespace VISG {

	// Bit set count operation from
	// http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
	int Matcher::DescriptorDistance(const cv::Mat &a, const cv::Mat &b)
	{
		
		const int *pa = a.ptr<int32_t>();
		const int *pb = b.ptr<int32_t>();
		//std::cout << "[Matcher DescriptorDistance] *pa: " << *pa << std::endl;
		int dist = 0;

		for (int i = 0; i<8; i++, pa++, pb++)
		{
			unsigned  int v = *pa ^ *pb;
			v = v - ((v >> 1) & 0x55555555);
			v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
			dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
		}

		return dist;
	}
	void Matcher::OrbMatch(std::vector<std::pair<int, int>>& matches, const  cv::Mat &descriptors1, const cv::Mat &descriptors2) {
		cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
		std::vector<cv::DMatch> d_matches;
		matcher->match(descriptors1, descriptors2, d_matches);
		auto min_dis = std::min_element(d_matches.begin(), d_matches.end(), [](const cv::DMatch& m1, const cv::DMatch& m2) {return m1.distance < m2.distance; })->distance;
		// std::cout << "[Matcher ]match::min_dis: " << min_dis << std::endl;
		matches.reserve(d_matches.size());
		for (size_t i = 0; i < d_matches.size(); ++i) {
			if (d_matches[i].distance <= std::max(static_cast<double>(2 * min_dis), 30.0)) {
				matches.push_back(std::make_pair(d_matches[i].queryIdx, d_matches[i].trainIdx));
			}
		}
	}
}