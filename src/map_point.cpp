#include"map_point.h"
namespace VISG {
	MapPoint::MapPoint(double x, double y, double z) {
		point = new double[3];
		point[0] = x;
		point[1] = y;
		point[2] = z;
		observed.reserve(Common::EveryNFrames);
	}

	MapPoint::MapPoint(const MapPoint &rhs) {
		*this = rhs;
	}

	MapPoint &MapPoint::operator = (const MapPoint &rhs) {
		if (this == &rhs)
			return *this;
		if (!rhs.point) {
			Clear();
			return *this;
		}
		double * tmp_point = new double[3];
		tmp_point[0] = rhs.point[0];
		tmp_point[1] = rhs.point[1];
		tmp_point[2] = rhs.point[2];
		delete[]point;
		point = tmp_point;
		observed = rhs.observed;
		return *this;
	}

	Eigen::Vector3f operator*(const Eigen::Matrix3f &R, const MapPoint &rhs) {
		return R * Eigen::Vector3f(rhs.point[0], rhs.point[1], rhs.point[2]);
	}

	MapPoint::~MapPoint() {
		Clear();
	}

	void MapPoint::BeObserved(size_t frame_id, size_t feature_id) {
		observed.push_back(std::make_pair(frame_id, feature_id));
	}

	Eigen::Vector3f operator-(const MapPoint &lhs, const Eigen::Vector3f &rhs) {
		return Eigen::Vector3f(lhs.point[0], lhs.point[1], lhs.point[2]) - rhs;
	}
}