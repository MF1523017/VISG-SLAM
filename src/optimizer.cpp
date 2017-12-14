#include "optimizer.h"
#include "utils.hpp"

namespace VISG {
	//R,t:brings points from the  camera coordinate system  to the world coordinate system
	// map_points : is in the world coordinate system
	float Optimizer::ProjectPoints(const std::vector<cv::Point3f> &map_points, const std::vector<cv::Point2f> &img_points, const Eigen::Matrix3f &R, const Eigen::Vector3f &t,
		std::vector<cv::Point2f> &pro_points) {
		if (map_points.empty())
			return -1;
		pro_points.clear();
		pro_points.reserve(map_points.size());
		const float fx = Common::Fx;
		const float fy = Common::Fy;
		const float cx = Common::Cx;
		const float cy = Common::Cy;
		float project_error = 0;
		for (size_t i = 0; i < map_points.size(); ++i) {
			Eigen::Vector3f pw(map_points[i].x, map_points[i].y, map_points[i].z);
			Eigen::Vector3f pc = R.transpose() * (pw -t);
			float u = fx * pc.x() / pc.z() + cx;
			float v = fy * pc.y() / pc.z() + cy;
			const cv::Point2f pro_point = cv::Point2f(u, v);
			const cv::Point2f error = pro_point - img_points[i];
			project_error += fabs(error.x) + fabs(error.y);
			pro_points.push_back(pro_point);
		}
		return project_error / map_points.size();
	}

	float Optimizer::ProjectPointsStereoMatch(const std::vector<MapPoint::Ptr>& map_points,
		const MatchPoints &match_points,
		const std::vector<bool> &inliers,
		const Eigen::Matrix3f &R, const Eigen::Vector3f &t,
		std::vector<cv::Point2f> &pro_points) {
		if (map_points.empty())
			return -1;
		pro_points.clear();
		pro_points.reserve(map_points.size());
		const float fx = Common::Fx;
		const float fy = Common::Fy;
		const float cx = Common::Cx;
		const float cy = Common::Cy;
		float project_error = 0;
		int counter = 0;
		for (size_t i = 0; i < map_points.size(); ++i) {
			if (!inliers[i])
				continue;
			
			Eigen::Vector3f pc = R.transpose() * (*map_points[i] - t);
			float lu = fx * pc.x() / pc.z() + cx;
			float lv = fy * pc.y() / pc.z() + cy;
			float ru = fx * (pc.x() - Common::BaseLine) / pc.z() + cx;
			const cv::Point2f pro_point = cv::Point2f(lu, lv);
			project_error += fabs(lu - match_points[i].x()) + fabs(lv - match_points[i].y()) + fabs(ru - match_points[i].z());
			pro_points.push_back(pro_point);
			++counter;
		}
		return project_error / counter;
	}

	float Optimizer::ProjectPointsRefTrack2D3D(const std::vector<MapPoint::Ptr>& map_points,
		const KeyPoints & key_points,
		const MyMatches &matches,
		const Eigen::Matrix3f &R, const Eigen::Vector3f &t,
		std::vector<cv::Point2f> &pro_points) {

		if (matches.empty())
			return -1;
		pro_points.clear();
		pro_points.reserve(map_points.size());
		const float fx = Common::Fx;
		const float fy = Common::Fy;
		const float cx = Common::Cx;
		const float cy = Common::Cy;
		float project_error = 0;
		for (size_t i = 0; i < matches.size(); ++i) {
			const size_t queryIdx = matches[i].first;
			const size_t trainIdx = matches[i].second;
			Eigen::Vector3f pc = R.transpose() * (*map_points[queryIdx] - t);
			if (pc.z() <= 0)
				continue;
			float u = fx * pc.x() / pc.z() + cx;
			float v = fy * pc.y() / pc.z() + cy;
			const cv::Point2f pro_point = cv::Point2f(u, v);
			const cv::Point2f error = pro_point - key_points[trainIdx].pt;
			project_error += fabs(error.x) + fabs(error.y);
			pro_points.push_back(pro_point);
		}
		return project_error / matches.size();
	}

	void PnpSolver::Solve(const std::vector<cv::Point2f>&points2, const std::vector<MapPoint::Ptr>&points3, cv::Mat &R, cv::Mat&t) {
		double *pose = new double[6]{0};
		std::cout << std::endl;
		ceres::LossFunction* loss_function = new ceres::HuberLoss(4);
		//double *p3d = new double[3 * points3.size()];
		for (size_t i = 0; i < points2.size(); ++i) {
		//	std::cout << "[PnpSolver::Solve] point3d: " << points3[i] << std::endl;
			/*p3d[3*i] = static_cast<double>(points3[i].x);
			p3d[3*i+1] = static_cast<double>(points3[i].y);
			p3d[3*i+2] = static_cast<double>(points3[i].z);*/
			ceres::CostFunction *cost_function = ProjectionError::Create(static_cast<double>(points2[i].x),
				static_cast<double>(points2[i].y),points3[i]->point );// p3d + 3 * i);
			
			problem_.AddResidualBlock(cost_function, loss_function, pose);
		}
		ceres::Solver::Options options;
		options.linear_solver_type = ceres::DENSE_SCHUR;
		options.minimizer_progress_to_stdout = true;
		ceres::Solver::Summary summary;
		ceres::Solve(options, &problem_, &summary);
		//std::cout << summary.FullReport() << std::endl;

		std::cout << std::endl << "[PnpSolver::Solve] pose after:";
		for (size_t i = 0; i < 6; ++i) {
			std::cout << pose[i] << " ";
		}
		std::cout << std::endl;
		R.at<float>(0, 0) = pose[0];
		R.at<float>(1, 0) = pose[1];
		R.at<float>(2, 0) = pose[2];
		t.at<float>(0, 0) = pose[3];
		t.at<float>(1, 0) = pose[4];
		t.at<float>(2, 0) = pose[5];
		delete[]pose;
		//delete[]p3d;
	}

	void BASolver::Solve(std::vector<Frame::Ptr> &local_frames, Frame::Ptr p_frame_ref) {
		// turn pose from Eigen to double *
		const size_t id_0 = p_frame_ref->id() + 1;
		double *poses = new double[6 * local_frames.size()]{0};
		for (size_t k = 0; k < local_frames.size(); ++k) {
			if (!local_frames[k])
				continue;
			std::cout << "[BASolver::Solve] wTc before BA: k: T" << k << std::endl<<local_frames[k]->wTc << std::endl;
			Eigen::Matrix4f cTr = HPose(local_frames[k]->rRc, local_frames[k]->rtc).inverse();
			cv::Mat R(3, 3, CV_64F);
			for (size_t i = 0; i < 3; ++i) {
				for (size_t j = 0; j < 3; ++j) {
					R.at<double>(i, j) = static_cast<double>(cTr(i, j));
				}
			}
			cv::Mat Rvec(3, 1, CV_64F);
			cv::Rodrigues(R, Rvec);
			poses[6 * k] = Rvec.at<double>(0, 0);
			poses[6 * k + 1] = Rvec.at<double>(1, 0);
			poses[6 * k + 2] = Rvec.at<double>(2, 0);
			poses[6 * k + 3] = static_cast<double>(cTr(0, 3));
			poses[6 * k + 4] = static_cast<double>(cTr(1, 3));
			poses[6 * k + 5] = static_cast<double>(cTr(2, 3));
		}
		
		// solve BA problem
		std::cout << "[BASolver::Solve] BA problem: " << std::endl;
		ceres::LossFunction* loss_function = new ceres::HuberLoss(4);
		auto mp_it = p_frame_ref->map_points.begin();
		for (; mp_it != p_frame_ref->map_points.end(); ++mp_it) {
			if (!(*mp_it))
				continue;
			for (MapPoint::Observed::iterator ob_it = (*mp_it)->observed.begin(); ob_it != (*mp_it)->observed.end(); ++ob_it) {
				const size_t idx = ob_it->first - id_0;
				const double x = static_cast<double>(local_frames[idx]->left_key_points[ob_it->second].pt.x);
				const double y = static_cast<double>(local_frames[idx]->left_key_points[ob_it->second].pt.y);
				ceres::CostFunction *cost_function = BAProjectionError::Create(x, y);
				problem_.AddResidualBlock(cost_function, loss_function, poses + 6 * idx,(*mp_it)->point);
			}
		}
		ceres::Solver::Options options;
		options.linear_solver_type = ceres::DENSE_SCHUR;
		options.minimizer_progress_to_stdout = true;
		ceres::Solver::Summary summary;
		ceres::Solve(options, &problem_, &summary);
		
		// turn pose from double * to Eigen
		for (size_t k = 0; k < local_frames.size(); ++k) {
			if (!local_frames[k])
				continue;
			cv::Mat Rvec(3, 1, CV_64F);
			Rvec.at<double>(0, 0) = poses[6 * k];
			Rvec.at<double>(1, 0) = poses[6 * k + 1];
			Rvec.at<double>(2, 0) = poses[6 * k + 2];
			cv::Mat R(3, 3, CV_64F);
			cv::Rodrigues(Rvec, R);
			Eigen::Matrix3f cRr = Rcv2Eigen(R);
			Eigen::Vector3f ctr(poses[6 * k + 3], poses[6 * k + 4], poses[6 * k + 5]);
			Eigen::Matrix4f rTc = HPose(cRr, ctr).inverse();
			HPose2Rt(rTc, local_frames[k]->rRc, local_frames[k]->rtc);
			local_frames[k]->wTc = p_frame_ref->wTc * rTc;
			std::cout << "[BASolver::Solve] wTc after BA: T" << k << std::endl<< local_frames[k]->wTc<< std::endl;
			HPose2Rt(local_frames[k]->wTc, local_frames[k]->wRc, local_frames[k]->wtc);
			
		}
	}
}