#include "frame.h"
#include "keyframe.h"
#include "matcher.h"
#include "utils.hpp"
#include <thread>

namespace VISG {
void Frame::SetPose(const Eigen::Matrix3f &R, const Eigen::Vector3f &t) {
	wRc = R;
	wTc = t;
}
void Frame::ExtractFeatures(const cv::Mat&left, const cv::Mat&right) {
	std::thread thread_left(&Frame::ExtractORB, this, left, true);
	std::thread thread_right(&Frame::ExtractORB, this, right, false);
	thread_left.join();
	thread_right.join();
	/*std::cout << "left_key_points size: " << left_key_points.size() << 
		" corrspondence left_descriptors cols : " << left_descriptors.cols << 
		" left_descriptors rows : "<< left_descriptors.row(0) << std::endl;*/
}
void Frame::ExtractORB(const cv::Mat&img, bool is_left) {
	if (is_left)
		p_orb_left_->Extract(img,left_key_points,left_descriptors);
	else
		p_orb_right_->Extract(img,right_key_points,right_descriptors);
}


void Frame::Match(KeyPoints &left_key_points, const cv::Mat &left_descriptors, KeyPoints &right_key_points, const cv::Mat&right_descriptors) {
	std::vector<std::vector<size_t>> left_hist,right_hist;
	match_points.clear();
	Hist(left_key_points, left_hist);
	Hist(right_key_points, right_hist);
	int pixel_diff = Common::Weight / 10;
	std::cout << "[Frame Match] pixel_diff: " << pixel_diff << std::endl;
	for (size_t i = 0; i < Common::HistBin; ++i) {
		if (left_hist[i].empty()|| right_hist[i].empty())
			continue;
		for (size_t j = 0; j < left_hist[i].size(); ++j) {
			size_t l_idx = left_hist[i][j];
			cv::Point2f &lp = left_key_points[l_idx].pt;
		//	std::cout << "[Frame Match] lp: " << lp << std::endl;
			int best_dist = 0x7fffffff;
			int best_r_idx = -1;
			cv::Point2f match_rp;
			for (size_t k = 0; k < right_hist[i].size(); ++k) {
				size_t r_idx = right_hist[i][k];
				cv::Point2f &rp = right_key_points[r_idx].pt;
		//		std::cout << "[Frame Match] rp: " << rp << std::endl;
				if (rp.x + 1 < 1e-3 || lp.x < rp.x || lp.x - rp.x > pixel_diff )
					continue;
				cv::Mat &ld = left_descriptors.row(l_idx);
				cv::Mat &rd = right_descriptors.row(r_idx);
				const int dist = Matcher::DescriptorDistance(ld, rd);
				if (dist < best_dist) {
					best_dist = dist;
					match_rp = rp;
					best_r_idx = r_idx;
				}
			}
			if (best_r_idx != -1) {
				std::cout << "[Frame Match] best_dist: " << best_dist << std::endl;
				match_points.push_back(Eigen::Vector3f(lp.x, lp.y, right_key_points[best_r_idx].pt.x));
				right_key_points[best_r_idx].pt.x = -1;
				std::cout << "[Frame Match] match_points: " << match_points.back() << std::endl;
			}
			
		}
	}
}

bool Frame::IsKeyFrame(MyMatches &matches) {
	return (matches.size() < KeyFrame::keys_th_tracked);
}
void Frame::Hist(KeyPoints &key_points, std::vector<std::vector<size_t>> &hist) {
	size_t step = Common::Height / Common::HistBin + 1;
	hist.resize(Common::HistBin);
	size_t point_size = key_points.size();
	for (size_t i = 0; i < point_size; ++i) {
		//std::cout << "[Frame Hist] y: " << key_points[i].pt.y << std::endl;
		size_t idx = floor(key_points[i].pt.y / step);
		//std::cout << "[Frame Hist] idx: " << idx << std::endl;
		hist[idx].push_back(i);
	}
}

void Frame::StereoMatch() {
	Reset();
	// histed right image key points by y
	int pixel_diff = Common::Weight / 10;
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
			match_points[i] = Eigen::Vector3f(lp.x, lp.y, rx);
			float z = Common::ltr.at<float>(0, 0) * Common::Fx / (lp.x - rx);
			const float x = (lp.x - Common::Cx)*z*Common::FxInv;
			const float y = (lp.y - Common::Cy)*z*Common::FyInv;
			Eigen::Vector3f x3Dc(x, y, z);
			map_points[i] = Eigen::Vector3f(wRc * x3Dc + wTc);
			inliers[i] = true;
		}
	}
}

bool Frame::RefTrack2D2D(Frame::Ptr p_frame_ref, MyMatches &inliers_matches) {
	MyMatches matches,matches1;
	inliers_matches.clear();
	// get the init matches
	Matcher::OrbMatch(matches, p_frame_ref->left_descriptors, left_descriptors);
	
	std::vector<cv::KeyPoint> & ref_key_points = p_frame_ref->left_key_points;
	const std::vector<bool> &ref_inliers = p_frame_ref->inliers;
	std::vector<cv::Point2f> points1;
	std::vector<cv::Point2f> points2;
	// apply for memory
	matches1.reserve(matches.size());
	points1.reserve(matches.size());
	points2.reserve(matches.size());
	for (size_t i = 0; i < matches.size(); ++i) {
		const int & queryIdx = matches[i].first;
		if (!ref_inliers[queryIdx])
			continue;
		const int & trainIdx = matches[i].second;
		points1.push_back(ref_key_points[queryIdx].pt);
		points2.push_back(left_key_points[trainIdx].pt);
		matches1.push_back(matches[i]);
	}
	if (points1.empty())
		return false;
	bool ret = RecoverPose(points1, points2, matches1,inliers_matches);
	if (!ret) {
		std::swap(inliers_matches, matches1);
		return false;
	}
	return true;
}

bool Frame::RecoverPose(const std::vector<cv::Point2f> &points1, const std::vector<cv::Point2f> &points2, const MyMatches &matches, MyMatches &inliers_matches) {
	cv::Mat mask;
	cv::Mat E = cv::findEssentialMat(points1, points2, Common::K, cv::RANSAC, 0.999, 1.0, mask);
	if (E.empty())
		return false;
	int valid_count = cv::countNonZero(mask);
	if (valid_count < 10 || static_cast<double>(valid_count) / points1.size() < 0.6)
		return false;
	inliers_matches.reserve(matches.size());
	for (size_t i = 0; i < mask.rows; ++i) {
		int status = mask.at<char>(i, 0);
		if (status) {
			inliers_matches.push_back(matches[i]);
		}
	}
	cv::Mat R, t;
	cv::recoverPose(E, points1, points2, Common::K, R, t, mask);// will change the status of the mask
	wRc = Rcv2Eigen(R);
	wTc = Tcv2Eigen(t);
	return true;
}

bool Frame::RefTrack2D3D(Frame::Ptr p_frame_ref, MyMatches &inliers_matches) {
	MyMatches matches, matches1;
	inliers_matches.clear();
	// get the init matches
	Matcher::OrbMatch(matches, p_frame_ref->left_descriptors, left_descriptors);

	std::vector<cv::KeyPoint> & ref_key_points = p_frame_ref->left_key_points;
	const std::vector<bool> &ref_inliers = p_frame_ref->inliers;
	const MapPoints &ref_map_points = p_frame_ref->map_points;
	std::vector<cv::Point2f> points2;
	std::vector<cv::Point3f> points3;
	// apply for memory
	matches1.reserve(matches.size());
	points2.reserve(matches.size());
	points3.reserve(matches.size());
	for (size_t i = 0; i < matches.size(); ++i) {
		const int & queryIdx = matches[i].first;
		if (!ref_inliers[queryIdx])
			continue;
		const int & trainIdx = matches[i].second;
		points2.push_back(left_key_points[trainIdx].pt);
		points3.push_back(PEigen2cv(ref_map_points[queryIdx]));
		matches1.push_back(matches[i]);
	}
	if (points2.empty())
		return false;
	bool ret = RecoverPose(points2, points3, matches1, inliers_matches);
	if (!ret) {
		std::swap(inliers_matches, matches1);
		return false;
	}
	return true;
}

bool Frame::RecoverPose(const std::vector<cv::Point2f> &points2, const std::vector<cv::Point3f> &points3, const MyMatches &matches, MyMatches &inliers_matches) {
	if (points2.size() != points3.size()) {
		std::cout << "[Frame::RecoverPose] correspondences error " << std::endl;
		return false;
	}
	cv::Mat Rvec,t,mask,R;
	// bool ret = cv::solvePnP(points3,points2,cam.K(),cv::Mat(),Rvec,t_,false,cv::SOLVEPNP_ITERATIVE);
	bool ret = cv::solvePnPRansac(points3, points2, Common::K, cv::Mat(), Rvec, t,true,100,8,0.99,mask);
	if (!ret) {
		std::cout << "[Frame::RecoverPose] pnp error" << std::endl;
		return false;
	}
	int valid_count = cv::countNonZero(mask);
	if (valid_count < 10 || static_cast<double>(valid_count) / points2.size() < 0.6)
		return false;
	inliers_matches.reserve(matches.size());
	for (size_t i = 0; i < mask.rows; ++i) {
		int status = mask.at<char>(i, 0);
		if (status) {
			inliers_matches.push_back(matches[i]);
		}
}
	cv::Rodrigues(Rvec, R);
	wRc = Rcv2Eigen(R);
	wTc = Tcv2Eigen(t);
	return true;
}

void Frame::Reset() {
	match_points.clear();//clear old match_points;
	map_points.clear();
	inliers.clear();
	inliers.resize(Common::FeaturesNum, false);
	match_points.resize(Common::FeaturesNum);
	map_points.resize(Common::FeaturesNum);
}


}