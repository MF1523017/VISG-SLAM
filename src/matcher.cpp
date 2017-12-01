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
}