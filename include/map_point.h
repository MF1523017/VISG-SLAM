#ifndef MAP_POINT_H
#define MAP_POINT_H
#include "common.h"
namespace VISG {
	class MapPoint {
	public:
		using Ptr = std::shared_ptr<MapPoint>;
		using Observed = std::vector<std::pair<size_t, size_t>>;
		MapPoint() :point(nullptr) {};
		MapPoint(double x, double y, double z);
		MapPoint(const MapPoint &rhs);
		MapPoint &operator = (const MapPoint &rhs);
		friend Eigen::Vector3f operator*(const Eigen::Matrix3f &R,const MapPoint &rhs);
		friend Eigen::Vector3f operator-(const MapPoint &lhs,const Eigen::Vector3f &rhs);
		~MapPoint();
		void Clear() {
			delete[]point;
			point = nullptr;
			observed.clear();
		}
		void BeObserved(size_t frame_id, size_t feature_id);
	public:
		double *point;
		Observed observed;
	};
	Eigen::Vector3f operator*(const Eigen::Matrix3f &R, const MapPoint &rhs);
	Eigen::Vector3f operator-(const MapPoint &lhs, const Eigen::Vector3f &rhs);
}

#endif
