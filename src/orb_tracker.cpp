#include "orb_tracker.h"
#include "draw_board.h"
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
		std::cout << "current frame key_points 0: " << p_frame_cur_->left_key_points[0].pt <<
			" last frame key_points 0: " << p_frame_last_->left_key_points[0].pt << std::endl;
		p_frame_last_ = p_frame_cur_;
	}
	bool OrbTracker::Init(cv::Mat &left, cv::Mat &right) {
		// TODO not defined
		p_frame_last_ = p_frame_ref_ = p_frame_cur_;
		p_frame_cur_->StereoMatch();
		DrawBoard::handle().DrawMatch(left, right, p_frame_cur_->match_points,false);
		return true;
	}
	bool OrbTracker::Track(cv::Mat &left, cv::Mat &right){
		// TODO not defined
		p_frame_cur_->StereoMatch();
		DrawBoard::handle().DrawMatch(left, right, p_frame_cur_->match_points, false);
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