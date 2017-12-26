#include"map_point.h"
namespace VISG {
	MapPoint::MapPoint(double x, double y, double z, unsigned char r, unsigned char g, unsigned char b) {
		point = new double[3];
		point[0] = x;
		point[1] = y;
		point[2] = z;

		color = new unsigned char[3];
		color[0] = r;
		color[1] = g;
		color[2] = b;

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
		if (!point) {
			point = new double[3];
			color = new unsigned char[3];
		}
		
		point[0] = rhs.point[0];
		point[1] = rhs.point[1];
		point[2] = rhs.point[2];

		color[0] = rhs.color[0];
		color[1] = rhs.color[1];
		color[2] = rhs.color[2];

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