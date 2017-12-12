#ifndef OPTIMIZER_H
#define OPTIMIZER_H
#include "common.h"
#include <ceres/ceres.h>
#include <ceres/rotation.h>
namespace VISG {
	class Optimizer {
	public:
		static float ProjectPoints(const std::vector<cv::Point3f> &map_points,
			const std::vector<cv::Point2f> &img_points,
			const Eigen::Matrix3f &R, const Eigen::Vector3f &t, 
			std::vector<cv::Point2f> &pro_points);
		static float ProjectPointsStereoMatch(const MapPoints& map_points,
			const MatchPoints &match_points,
			const std::vector<bool> &inliers,
			const Eigen::Matrix3f &R, const Eigen::Vector3f &t,
			std::vector<cv::Point2f> &pro_points);
		static float ProjectPointsRefTrack2D3D(const MapPoints& map_points, 
			const KeyPoints & key_points,
			const MyMatches &matches,
			const Eigen::Matrix3f &R, const Eigen::Vector3f &t,
			std::vector<cv::Point2f> &pro_points);

};
	struct ProjectionError {
		ProjectionError(double observed_x, double observed_y)
			: observed_x(observed_x), observed_y(observed_y) {}

		template <typename T>
		bool operator()(const T *const pose,
			const T* const point,
			T* residuals) const {
			// camera[0,1,2] are the angle-axis rotation.
			T p[3];
			ceres::AngleAxisRotatePoint(pose, point, p);

			// camera[3,4,5] are the translation.
			p[0] += pose[3];
			p[1] += pose[4];
			p[2] += pose[5];

			// Compute the center of distortion. The sign change comes from
			// the camera model that Noah Snavely's Bundler assumes, whereby
			// the camera coordinate system has a negative z axis.
			T xp = p[0] / p[2];
			T yp = p[1] / p[2];
			
			//// Apply second and fourth order radial distortion.
			//const T& l1 = camera[7];
			//const T& l2 = camera[8];
			//T r2 = xp*xp + yp*yp;
			//T distortion = 1.0 + r2  * (l1 + l2  * r2);

			// Compute final projected point position.
			/*const T& fx = camera[];
			T predicted_x = focal * distortion * xp;
			T predicted_y = focal * distortion * yp;*/
			const T  fx = static_cast<T>(Common::Fx);
			const T  fy = static_cast<T>(Common::Fy);
			const T  cx = static_cast<T>(Common::Cx);
			const T  cy = static_cast<T>(Common::Cy);
			T predicted_x = fx * xp + cx;
			T predicted_y = fy * yp + cy; 
			
			// The error is the difference between the predicted and observed position.
			residuals[0] = predicted_x - observed_x;
			residuals[1] = predicted_y - observed_y;
			return true;
		}

		// Factory to hide the construction of the CostFunction object from
		// the client code.
		static ceres::CostFunction* Create(const double observed_x,
			const double observed_y) {
			return (new ceres::AutoDiffCostFunction<ProjectionError, 2,6,3>(
				new ProjectionError(observed_x, observed_y)));
		}
		
		double observed_x;
		double observed_y;
	};
	class PnpSolver {
	public:
		PnpSolver() = default;
		void Solve(const std::vector<cv::Point2f>&points2, const std::vector<cv::Point3f>&points3, cv::Mat &R, cv::Mat&t);
	private:
		ceres::Problem problem_;
	};
}
#endif