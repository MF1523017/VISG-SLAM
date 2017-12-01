#include "frame.h"
#include "keyframe.h"
#include "matcher.h"
#include <thread>

namespace VISG {
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
				match_points.push_back(cv::Point3f(lp.x, lp.y, right_key_points[best_r_idx].pt.x));
				right_key_points[best_r_idx].pt.x = -1;
				std::cout << "[Frame Match] match_points: " << match_points.back() << std::endl;
			}
			
		}
	}
}

bool Frame::IsKeyFrame(KeyPoints &key_points) {
	return (key_points.size() < KeyFrame::keys_th_tracked) && (key_points.size() > KeyFrame::keys_th_created);
}
void Frame::Hist(KeyPoints &key_points, std::vector<std::vector<size_t>> &hist) {
	size_t step = Common::Height / Common::HistBin + 1;
	
	std::cout << "[Frame Hist] step: " << step << std::endl;
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
	match_points.clear();//clear old match_points;
	int pixel_diff = Common::Weight / 10;
	std::vector<std::vector<size_t>> r_hist(Common::HistBin,std::vector<size_t>());
	size_t step = Common::Height / Common::HistBin + 1;
	size_t point_size = right_key_points.size();
	for (size_t i = 0; i < point_size; ++i) {
		size_t idx = floor(right_key_points[i].pt.y / step);
		r_hist[idx].push_back(i);
	}

	size_t keys_size = left_key_points.size();
	
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
	//		std::cout << "[Frame Match] best_dist: " << best_dist << std::endl;
			match_points.push_back(cv::Point3f(lp.x, lp.y, right_key_points[best_r_idx].pt.x));
	//		std::cout << "[Frame Match] match_points: " << match_points.back() << std::endl;
		}
	}
}



}