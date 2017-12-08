#include "orb_tracker.h"
#include "draw_board.h"
#include "utils.hpp"
namespace VISG {
	OrbTracker::OrbTracker() :TrackerInterface(),p_frame_cur_(new Frame), p_frame_last_(new Frame), p_frame_ref_(new Frame) {
		;
	}
	void OrbTracker::operator()(cv::Mat &left, cv::Mat &right) {
		p_frame_cur_ = std::make_shared<Frame>();
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
		p_frame_last_ = p_frame_cur_;
	}
	bool OrbTracker::Init(cv::Mat &left, cv::Mat &right) {
		p_frame_cur_->StereoMatch();
		if (p_frame_cur_->match_points.size() > 100) {
			p_frame_last_ = p_frame_ref_ = p_frame_cur_;
			ref_image = left.clone();
			std::cout << "init sucessful! " << std::endl;
		//	DrawBoard::handle().DrawMatch(left, right, p_frame_cur_->match_points, false);
			return true;
		}
		return false;
	}
	bool OrbTracker::Track(cv::Mat &left, cv::Mat &right){
		// TODO not defined
	//	std::cout << "[OrbTracker Track] p_frame_ref_ status: before " << p_frame_ref_.use_count() << std::endl;
		MyMatches my_matches;
		// recover pose using  2d to 2d corrspondence 
		auto ret = p_frame_cur_->RefTrack2D2D(p_frame_ref_, my_matches);
		std::cout << "[OrbTracker Track] RefTrack2D2D ret: " << ret << " mathches size: " << my_matches.size() << std::endl;
		// recover pose using 2d to 3d corrspondence
		/*auto ret = p_frame_cur_->RefTrack2D3D(p_frame_ref_, my_matches);
		std::cout << "[OrbTracker Track] RefTrack2D3D ret: " << ret << " mathches size: " << my_matches.size() << std::endl;*/
		DrawBoard::handle().DrawPose(left, p_frame_cur_->wRc, p_frame_cur_->wTc, ret);
		DrawBoard::handle().DrawMatch(ref_image, left, my_matches, p_frame_ref_->left_key_points, p_frame_cur_->left_key_points, false);
		
		if (ret && p_frame_cur_->IsKeyFrame(my_matches)) {
			p_frame_cur_->StereoMatch();
			p_frame_ref_.swap(p_frame_cur_);// = p_frame_cur_;
			ref_image = left.clone();
		}
	//	std::cout << "[OrbTracker Track] p_frame_ref_ status: after " << p_frame_ref_.use_count() << std::endl;
		return true;
	}
	std::vector<cv::Point3f> OrbTracker::GetMapPoints()const {
		// TODO not defined
		return std::vector<cv::Point3f>();
	}
	void OrbTracker::Reboot() {
		// TODO not defined
		;
	}
	int OrbTracker::GetPose(Eigen::Matrix3f& R, Eigen::Vector3f &t) const {
		// TODO not defined
		R = p_frame_cur_->wRc;
		t = p_frame_cur_->wTc;
		return 0;
	}
}