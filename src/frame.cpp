#include "frame.h"
#include "keyframe.h"
#include "matcher.h"
#include "utils.hpp"
#include "optimizer.h"
#include <thread>

namespace VISG {
void Frame::SetPose(const Eigen::Matrix3f &R, const Eigen::Vector3f &t) {
	wRc = R;
	wtc = t;
	wTc = HPose(R, t);

}
void Frame::ExtractFeatures(const cv::Mat&left, const cv::Mat&right) {
	std::thread thread_left(&Frame::ExtractORB, this, left, true);
	std::thread thread_right(&Frame::ExtractORB, this, right, false);
	thread_left.join();
	thread_right.join();
}
void Frame::ExtractORB(const cv::Mat&img, bool is_left) {
	if (is_left)
		p_orb_left_->Extract(img,left_key_points,left_descriptors);
	else
		p_orb_right_->Extract(img,right_key_points,right_descriptors);
}



bool Frame::IsKeyFrame(MyMatches &matches) {
	return (matches.size() < KeyFrame::keys_th_tracked);
}


size_t Frame::StereoMatch(const cv::Mat &left) {
	size_t matches_counter = 0;
	Reset();
	// histed right image key points by y
	int pixel_diff = Common::Width / 10;
	std::vector<std::vector<size_t>> r_hist(Common::HistBin,std::vector<size_t>());
	for (size_t i = 0; i < Common::HistBin; ++i) {
		r_hist[i].reserve(200);
	}
	size_t step = Common::Height / Common::HistBin + 1;
	const size_t point_size = right_key_points.size();
	for (size_t i = 0; i < point_size; ++i) {
		size_t idx = floor(right_key_points[i].pt.y / step);
		r_hist[idx].push_back(i);
	}

	// get the match points 
	const size_t keys_size = left_key_points.size();
	for (size_t i = 0; i < keys_size; ++i) {
		const cv::Point2f &lp = left_key_points[i].pt;
		size_t idx = floor(lp.y / step);
		if (r_hist.empty())
			continue;
		const std::vector<size_t> &r_candidates = r_hist[idx];
		int best_dist = 0x7fffffff;
		int best_r_idx = -1;
		for (size_t j = 0; j < r_candidates.size(); ++j) {
			const size_t r_idx = r_candidates[j];
			const cv::Point2f &rp = right_key_points[r_idx].pt;
			if (rp.x < lp.x && lp.x - rp.x < pixel_diff) {
				cv::Mat &ld = left_descriptors.row(i);
				cv::Mat &rd = right_descriptors.row(r_idx);
				const int dist = Matcher::DescriptorDistance(ld, rd);
				if (dist < best_dist) {
					best_dist = dist;
					best_r_idx = r_idx;
				}
			}
		}
		if (best_dist < Common::BestOrbDistance) {
			const float  rx = right_key_points[best_r_idx].pt.x;
			match_points[i] = Eigen::Vector3d(lp.x, lp.y, rx);
			double z = Common::BaseLine * Common::Fx / (lp.x - rx);
			if (z <= 0 || z < 0.5 || z >15)
				continue;
			const double x = (lp.x - Common::Cx)*z*Common::FxInv;
			const double y = (lp.y - Common::Cy)*z*Common::FyInv;
			const int u = floor(lp.x);
			const int v = floor(lp.y);
			//std::cout << "[Frame::StereoMatch] u: " << u << ", v: " << v << std::endl;
			map_points[i] = std::make_shared<MapPoint>(x,y,z, 
				left.at<cv::Vec3b>(v, u)[0], 
				left.at<cv::Vec3b>(v, u)[1],
				left.at<cv::Vec3b>(v, u)[2]);
			
			inliers[i] = true;
			++matches_counter;
		}
	}
	return matches_counter;
}

bool Frame::RefTrack2D2D(Frame::Ptr p_frame_ref, MyMatches &inliers_matches) {
	bool ret;
	ret = FetchMatches(p_frame_ref, inliers_matches);
	if (!ret) {
		std::cout << "[Frame::RefTrack2D3D]: fetch matches error!" << std::endl;
		return false;
	}
	cv::Mat R(3, 3, CV_32F), t(3, 1, CV_32F);
	ret = RecoverPose(p_frame_ref, inliers_matches,R,t);
	if (!ret) {
		std::cout << "[Frame::RefTrack2D3D]: RecoverPose error!" << std::endl;
		return false;
	}
	 
	Eigen::Matrix3f cRr = Rcv2Eigen(R);//ref to cur
	Eigen::Vector3f ctr = Tcv2Eigen(t);
	Eigen::Matrix4f rTc = HPose(cRr, ctr).inverse();
	Eigen::Matrix4f wTr = p_frame_ref->wTc;
	wTc = wTr * rTc;
	HPose2Rt(wTc, wRc, wtc);
	HPose2Rt(rTc, rRc, rtc);
	return true;
}


bool Frame::RecoverPose(Frame::Ptr p_frame_ref, MyMatches &inliers_matches, cv::Mat &R, cv::Mat &t) {
	std::vector<cv::Point2f> points_ref, points_cur;
	points_cur.reserve(inliers_matches.size());
	points_ref.reserve(inliers_matches.size());
	for (auto it = inliers_matches.begin(); it != inliers_matches.end(); ++it) {
		points_ref.push_back(p_frame_ref->left_key_points[it->first].pt);
		points_cur.push_back(left_key_points[it->second].pt);
	}
	cv::Mat mask;
	cv::Mat E = cv::findEssentialMat(points_ref, points_cur, Common::K, cv::RANSAC, 0.999, 1.0, mask);
	if (E.empty())
		return false;
	int valid_count = cv::countNonZero(mask);
	if (valid_count < 10 || static_cast<double>(valid_count) / points_cur.size() < 0.6)
		return false;

	cv::recoverPose(E, points_ref, points_cur, Common::K, R, t, mask);// will change the status of the mask
	return true;
}

bool Frame::FetchMatches(Frame::Ptr p_frame_ref, MyMatches &inliers_matches) {
	MyMatches matches, matches1;
	// get the init matches
	Matcher::OrbMatch(matches, p_frame_ref->left_descriptors, left_descriptors);
	std::cout << "[Frame::FetchMatchPoints]: orbmatches size: " << matches.size() << std::endl;
	const std::vector<cv::KeyPoint> & ref_key_points = p_frame_ref->left_key_points;
	const std::vector<bool> &ref_inliers = p_frame_ref->inliers;
	std::vector<MapPoint::Ptr> &ref_map_points = p_frame_ref->map_points;
	std::vector<cv::Point2f> points_ref; // ref is prev frame
	std::vector<cv::Point2f> points_cur;

	// apply for memory
	matches1.reserve(matches.size());
	points_ref.reserve(matches.size());
	points_cur.reserve(matches.size());

	for (size_t i = 0; i < matches.size(); ++i) {
		const int & queryIdx = matches[i].first;
		const int & trainIdx = matches[i].second;
		if (!ref_inliers[queryIdx] || !inliers[trainIdx])
			continue;
		points_ref.push_back(ref_key_points[queryIdx].pt);
		points_cur.push_back(left_key_points[trainIdx].pt);
		matches1.push_back(matches[i]);
	}
	if (matches1.size() < 3) {
		std::cout << "[Frame::RefTrack2D3D] error: points size is too less: " << matches1.size() << std::endl;
		return false;
	}

	cv::Mat mask;
	cv::Mat E = cv::findEssentialMat(points_ref, points_cur, Common::K, cv::RANSAC, 0.999, 1.0, mask);
	if (E.empty()) {
		std::cout << "[Frame::RefTrack2D3D] error: E is empty: " << std::endl;
		return false;
	}
	int valid_count = cv::countNonZero(mask);
	if (valid_count < 10 || static_cast<double>(valid_count) / points_ref.size() < 0.6) {
		std::cout << "[Frame::RefTrack2D3D] error: valid_count : " << valid_count << std::endl;
		return false;
	}
	inliers_matches.reserve(matches1.size());
	for (size_t i = 0; i < mask.rows; ++i) {
		int status = mask.at<char>(i, 0);
		if (status) {
			const int & queryIdx = matches1[i].first;
			const int & trainIdx = matches1[i].second;
			inliers_matches.push_back(matches1[i]);
			ref_map_points[queryIdx]->BeObserved(id_, trainIdx);
		}
	}
	return true;
}

bool Frame::RefTrack2D3D(Frame::Ptr p_frame_ref, MyMatches &inliers_matches) {
	bool ret;
	ret = FetchMatches(p_frame_ref, inliers_matches);
	if (!ret) {
		std::cout << "[Frame::RefTrack2D3D]: fetch match points error!" << std::endl;
		return false;
	}
	cv::Mat R(3,3,CV_32F), t(3,1,CV_32F);
	
	/*ret = RecoverPoseWithcvPnp(p_frame_ref, inliers_matches, R, t);
	if (!ret) {
		std::cout << "[Frame::RefTrack2D3D]: RecoverPose error!" << std::endl;
		return false;
	}*/
	
	ret = RecoverPoseWithPnpSolver(p_frame_ref, inliers_matches, R, t);
	//ret = RecoverPoseWithStereoMatchesPnp(p_frame_ref, inliers_matches, R, t);
	Eigen::Matrix3f cRr = Rcv2Eigen(R);//ref to cur
	Eigen::Vector3f ctr = Tcv2Eigen(t);
	Eigen::Matrix4f rTc = HPose(cRr, ctr).inverse();
	Eigen::Matrix4f wTr = p_frame_ref->wTc;
	wTc = wTr * rTc;
	HPose2Rt(wTc, wRc, wtc);
	HPose2Rt(rTc, rRc, rtc);
	return true;
}

bool Frame::RecoverPoseWithcvPnp(Frame::Ptr p_frame_ref, MyMatches &inliers_matches, cv::Mat &R, cv::Mat &t) {
	std::vector<cv::Point3f> points3;
	std::vector<cv::Point2f> points2;
	points3.reserve(inliers_matches.size());
	points2.reserve(inliers_matches.size());
	for (auto it = inliers_matches.begin(); it != inliers_matches.end(); ++it) {
		points3.push_back(Pdouble2cv(p_frame_ref->map_points[it->first]->point));
		points2.push_back(left_key_points[it->second].pt);
	}
	cv::Mat Rvec(3,1,CV_32F),mask;
	// R t: brings points from the model coordinate system to the camera coordinate system
	bool ret = cv::solvePnPRansac(points3, points2, Common::K, cv::Mat(), Rvec, t,true,100,8,0.99,mask);
	if (!ret) {
		std::cout << "[Frame::RecoverPose] pnp error" << std::endl;
		return false;
	}
	cv::Rodrigues(Rvec, R);

#ifdef USE_PROJECT_ERROR
	std::vector<cv::Point2f> pro_points;
	cv::projectPoints(points3, Rvec, t, Common::K, cv::Mat(), pro_points);
	float pro_error = 0;
	for (size_t i = 0; i < points2.size(); ++i) {
		cv::Point2f error = points2[i] - pro_points[i];
		pro_error += (fabs(error.x) + fabs(error.y));
	}
	std::cout << "[Frame::RecoverPose] pro_error: " << pro_error / points2.size() << std::endl;

#endif

	return true;
}


bool Frame::RecoverPoseWithPnpSolver(Frame::Ptr p_frame_ref, MyMatches &inliers_matches, cv::Mat &R, cv::Mat &t) {
	std::vector<MapPoint::Ptr> mp_p3ds;
	mp_p3ds.reserve(inliers_matches.size());
	std::vector<cv::Point2f> points2;
	points2.reserve(inliers_matches.size());
	std::vector<cv::Point3f> points3;
	for (auto it = inliers_matches.begin(); it != inliers_matches.end(); ++it) {
		mp_p3ds.push_back(p_frame_ref->map_points[it->first]);
		points2.push_back(left_key_points[it->second].pt);
		points3.push_back(Pdouble2cv(p_frame_ref->map_points[it->first]->point));
	}
	cv::Mat Rvec(3, 1, CV_32F);
	PnpSolver pnp_solver;
	pnp_solver.Solve(mp_p3ds,points2, Rvec,t);
	cv::Rodrigues(Rvec, R);
//	std::cout << "[Frame::RecoverPoseWithPnpSolver] R: " << R << std::endl << "t: " << t << std::endl;
#ifdef USE_PROJECT_ERROR
	std::vector<cv::Point2f> pro_points;
	cv::projectPoints(points3, Rvec, t, Common::K, cv::Mat(), pro_points);
	float pro_error = 0;
	for (size_t i = 0; i < points2.size(); ++i) {
		cv::Point2f error = points2[i] - pro_points[i];
		pro_error += (fabs(error.x) + fabs(error.y));
	}
	std::cout << "[Frame::RecoverPoseWithPnpSolver] pro_error: " << pro_error / points2.size() << std::endl;
#endif
	return true;
}

bool Frame::RecoverPoseWithStereoMatchesPnp(Frame::Ptr p_frame_ref, MyMatches &inliers_matches, cv::Mat &R, cv::Mat &t) {
	std::vector<MapPoint::Ptr> mp_p3ds;
	mp_p3ds.reserve(inliers_matches.size());
	MatchPoints stereo_points;
	stereo_points.reserve(inliers_matches.size());
	std::vector<cv::Point3f> points3;
	for (auto it = inliers_matches.begin(); it != inliers_matches.end(); ++it) {
		mp_p3ds.push_back(p_frame_ref->map_points[it->first]);
		stereo_points.push_back(match_points[it->second]);
		points3.push_back(Pdouble2cv(p_frame_ref->map_points[it->first]->point));
	}
	cv::Mat Rvec(3, 1, CV_32F);
	StereoPnpSolver pnp_solver;
	pnp_solver.Solve(mp_p3ds, stereo_points, Rvec, t);
	cv::Rodrigues(Rvec, R);
	//	std::cout << "[Frame::RecoverPoseWithPnpSolver] R: " << R << std::endl << "t: " << t << std::endl;
#ifdef USE_PROJECT_ERROR
	std::vector<cv::Point2f> pro_points;
	//cv::projectPoints(points3, Rvec, t, Common::K, cv::Mat(), pro_points);
	float pro_error = 0;
	for (size_t i = 0; i < inliers_matches.size(); ++i) {
		;
	}
	std::cout << "[Frame::RecoverPoseWithPnpSolver] pro_error: " << pro_error / inliers_matches.size() << std::endl;
#endif
	return true;
}


void Frame::Reset() {
	match_points.clear();//clear old match_points;
	map_points.clear();
	inliers.clear();
	inliers.resize(Common::FeaturesNum, false);
	match_points.resize(Common::FeaturesNum);
	//map_points.resize(Common::FeaturesNum);
	map_points.resize(Common::FeaturesNum, nullptr);
}
void Frame::GetwMapPoints(std::vector<Eigen::Vector3f> &valid_map_points) {
	valid_map_points.clear();
	valid_map_points.reserve(Common::FeaturesNum);
	for (size_t i = 0; i < inliers.size(); ++i) {
		if (inliers[i]) {
			valid_map_points.push_back(wRc * (*map_points[i]) + wtc);;
		}
	}
}

void Frame::MotionTrack(Frame::Ptr p_frame_ref) {
	rRc = p_frame_ref->rRc;
	rtc = p_frame_ref->rtc;
	Eigen::Matrix4f rTc = HPose(rRc, rtc);
	Eigen::Matrix4f wTr = p_frame_ref->wTc;
	wTc = wTr * rTc;
	HPose2Rt(wTc, wRc, wtc);
}


Frame::~Frame() {
	left_descriptors.release();
	right_descriptors.release();
	left_key_points.clear();
	right_key_points.clear();
	match_points.clear();
	map_points.clear();
	inliers.clear();

}

}