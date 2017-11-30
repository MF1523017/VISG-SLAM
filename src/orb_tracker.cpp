#include "orb_tracker.h"
#include "draw_board.h"
namespace VISG {
	OrbTracker::OrbTracker() :TrackerInterface(),p_frame_cur_(new Frame), p_frame_last_(new Frame), p_frame_ref_(new Frame) {
		;
	}
	void OrbTracker::operator()(cv::Mat &left, cv::Mat &right) {
		p_frame_cur_->ExtractFeatures(left, right);
	//	DrawBoard::handle().DrawFeatures(right, p_frame_cur_->right_key_points, false);
		// TODO 
		switch (state_)
		{
		case VISG::TrackerInterface::INIT:
			Init(left,right);
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
	}
	bool OrbTracker::Init(cv::Mat &left, cv::Mat &right) {
		// TODO not defined
		p_frame_last_ = p_frame_ref_ = p_frame_cur_;
		KeyPoints & lkeys = p_frame_cur_->left_key_points;
		cv::Mat & ld = p_frame_cur_->left_descriptors;
		KeyPoints & rkeys = p_frame_cur_->right_key_points;
		cv::Mat & rd = p_frame_cur_->right_descriptors;
		p_frame_cur_->Match(lkeys, ld, rkeys, rd);
		DrawBoard::handle().DrawMatch(left, right, p_frame_cur_->match_points);
		return true;
	}
	bool OrbTracker::Track(cv::Mat &left, cv::Mat &right){
		// TODO not defined
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
	int OrbTracker::GetPose(cv::Mat&R, cv::Mat &t) const {
		// TODO not defined
		return 0;
	}
}