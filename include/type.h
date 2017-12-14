#ifndef TYPE_H
#define TYPE_H
#include "common.h"
#include <Eigen/Dense>
namespace VISG {
	using KeyPoints = std::vector<cv::KeyPoint>;
	using MatchPoints = std::vector<Eigen::Vector3f>;
	using MyMatches = std::vector<std::pair<int, int>>;
}

#endif
