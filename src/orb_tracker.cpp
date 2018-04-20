#include "orb_tracker.h"
#include "draw_board.h"
#include "utils.hpp"
#include "optimizer.h"
namespace VISG {
	OrbTracker::OrbTracker() :TrackerInterface(),p_frame_cur_(nullptr), p_frame_last_(nullptr), 
		p_frame_ref_(nullptr), motion_counter_(0), frame_id_(0){
		local_frames_.resize(Common::EveryNFrames,nullptr);
	}

	void OrbTracker::operator()(cv::Mat &left, cv::Mat &right) {
		p_frame_cur_ = std::make_shared<Frame>(frame_id_);
		p_frame_cur_->ExtractFeatures(left, right);
		
	//	DrawBoard::handle().DrawFeatures(right, p_frame_cur_->right_key_points, false);
		// TODO ,
		switch (state_)
		{
		case VISG::TrackerInterface::INIT:
			if(Init(left,right))
				state_ = TRACKING;
			break;
		case VISG::TrackerInterface::TRACKING:
			Track(left, right);
			break;
		case VISG::TrackerInterface::LOST:
			Reboot();
			break;
		default:
			break;
		}
		/*std::cout << "current frame key_points 0: " << p_frame_cur_->left_key_points[0].pt <<
			" last frame key_points 0: " << p_frame_last_->left_key_points[0].pt << std::endl;*/
		
	}

	bool OrbTracker::Init(cv::Mat &left, cv::Mat &right) {
		size_t ret = p_frame_cur_->StereoMatch(left);
		std::cout << "[OrbTracker::Init] init .....: " << ret << std::endl;
		if (ret > 60) {
			std::cout << "[OrbTracker::Init] mathches size: " << ret << std::endl;
			p_frame_last_ = p_frame_ref_ = p_frame_cur_;
			ref_image = left.clone();
			++frame_id_;
			std::cout << "[OrbTracker::Init] init sucessful! " << std::endl;
#ifdef USE_PROJECT_ERROR
			std::vector<cv::Point2f> pro_points;
			float e = Optimizer::ProjectPointsStereoMatch(p_frame_cur_->map_points, 
				p_frame_cur_->match_points, p_frame_cur_->inliers, p_frame_cur_->rRc, p_frame_cur_->rtc, pro_points);
			std::cout << "[OrbTracker::Init] project error: " << e << std::endl;
#endif

			DrawBoard::handle().DrawMatch(left, right, p_frame_cur_->match_points);
			return true;
		}
		return false;
	}

	bool OrbTracker::Track(cv::Mat &left, cv::Mat &right){
		// TODO not defined
	//	std::cout << "[OrbTracker Track] p_frame_ref_ status: before " << p_frame_ref_.use_count() << std::endl;
		p_frame_cur_->StereoMatch(left);
		MyMatches my_matches;
		bool ret;
		// recover pose using  2d to 2d corrspondence 
		/*auto ret = p_frame_cur_->RefTrack2D2D(p_frame_ref_, my_matches);
		std::cout << "[OrbTracker Track] RefTrack2D2D ret: " << ret << " mathches size: " << my_matches.size() << std::endl;*/
		
		// recover pose using 2d to 3d corrspondence
		 ret = p_frame_cur_->RefTrack2D3D(p_frame_ref_, my_matches);
		
		// recover pose using 3d to 3d corrspondence (icp)
		//ret = p_frame_cur_->RefTrack3D3D(p_frame_ref_, my_matches);
	
		if (!ret) {
			p_frame_cur_->MotionTrack(p_frame_last_);
			std::cout << "[OrbTracker Track] MotionTrack matches size: " << my_matches.size() << std::endl;
			//cv::waitKey();
			if (30 == ++motion_counter_){
				state_ = LOST;
			}
		}
		p_frame_last_ = p_frame_cur_;
		const size_t diff_id = p_frame_cur_->id() - p_frame_ref_->id()-1;
		//local_frames_[diff_id] = p_frame_cur_;
		motion_counter_ = 0;
		cv::Mat tmp(left.size(), left.type(),cv::Scalar::all(0)), left1(left.size(),left.type(), cv::Scalar::all(0));
#ifdef USE_PROJECT_ERROR
		std::vector<cv::Point2f> pro_points;
		float e = Optimizer::ProjectPointsRefTrack2D3D(p_frame_ref_->map_points,
			p_frame_cur_->left_key_points, my_matches, p_frame_cur_->rRc, p_frame_cur_->rtc, pro_points);
		std::cout << "[OrbTracker Track] Project error: " << e << std::endl;
		DrawBoard::handle().DrawProjectError(tmp, p_frame_cur_->left_key_points, my_matches, pro_points);
#endif

	// drawing
#ifdef DRAW	
		//DrawBoard::handle().DrawFeatures(left, p_frame_cur_->left_key_points,false);
		cv::Mat ar(left.size(), left.type(), cv::Scalar::all(0)),left_ar(left.size(), left.type(), cv::Scalar::all(0));
		std::vector<Eigen::Vector3f> map_points;
		p_frame_ref_->GetwMapPoints(map_points);
		DrawBoard::handle().DrawAR(ar, map_points, p_frame_cur_->wRc, p_frame_cur_->wtc);
		DrawBoard::handle().DrawPose(tmp,p_frame_cur_->wRc, p_frame_cur_->wtc, ret);
		DrawBoard::handle().DrawPose(ar, p_frame_cur_->wRc, p_frame_cur_->wtc, ret);
		left1 = left + tmp;
		left_ar = left + ar;
		DrawBoard::handle().ShowAR(left_ar);
		DrawBoard::handle().DrawMatch(ref_image, left1, my_matches, p_frame_ref_->left_key_points, p_frame_cur_->left_key_points);
#endif	
		if (0 == ((frame_id_++) % Common::EveryNFrames)) {
			/*BAOnlyPointsSolver ba_only_points_solver;
			ba_only_points_solver.Solve(local_frames_, p_frame_ref_);
			BAOnlyPosesSolver ba_only_pose_solver;
			ba_only_pose_solver.Solve(local_frames_, p_frame_ref_);
			std::vector<cv::Point2f> pro_points;
			float e = Optimizer::ProjectPointsRefTrack2D3D(p_frame_ref_->map_points,
				p_frame_cur_->left_key_points, my_matches, p_frame_cur_->rRc, p_frame_cur_->rtc, pro_points);
			std::cout << "[OrbTracker Track] Project error after BA: " << e << std::endl;*/
			/*local_frames_.clear();
			local_frames_.resize(Common::EveryNFrames, nullptr);*/
			
			p_frame_ref_ = p_frame_cur_;
			ref_image = left.clone();
		}
	//	std::cout << "[OrbTracker Track] p_frame_ref_ status: after " << p_frame_ref_.use_count() << std::endl;
		
		return true;
	}

	void OrbTracker::GetMapPoints(std::vector<Eigen::Vector3f> &map_points, std::vector<Eigen::Vector3i> &colors) {
		// TODO not defined
		p_frame_cur_->GetwMapPoints(map_points);
		colors.clear();
		colors.reserve(map_points.size());
		for (const MapPoint::Ptr p : p_frame_cur_->map_points) {
			if (!p)
				continue;
			colors.push_back(Eigen::Vector3i(p->color[0], p->color[1], p->color[2]));
		}
		
	}

	void OrbTracker::Reboot() {
		p_frame_cur_ = p_frame_last_ = p_frame_ref_ = nullptr;
		motion_counter_ = 0;
		frame_id_ = 0;
		local_frames_.clear();
		ref_image.release();
	}

	int OrbTracker::GetPose(Eigen::Matrix3f& R, Eigen::Vector3f &t) const {
		// TODO not defined
		R = p_frame_cur_->wRc;
		t = p_frame_cur_->wtc;
		return 0;
	}
}