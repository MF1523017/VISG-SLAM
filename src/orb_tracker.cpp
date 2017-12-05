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
		/*std::cout << "current frame key_points 0: " << p_frame_cur_->left_key_points[0].pt <<
			" last frame key_points 0: " << p_frame_last_->left_key_points[0].pt << std::endl;*/
		p_frame_last_ = p_frame_cur_;
	}
	bool OrbTracker::Init(cv::Mat &left, cv::Mat &right) {
		p_frame_last_ = p_frame_ref_ = p_frame_cur_;
		ref_image = left.clone();
		p_frame_cur_->StereoMatch();
		DrawBoard::handle().DrawMatch(left, right, p_frame_cur_->match_points,false);
		return true;
	}
	bool OrbTracker::Track(cv::Mat &left, cv::Mat &right){
		// TODO not defined
		MyMatches my_matches;
		auto ret = p_frame_cur_->RefTrack(p_frame_ref_, my_matches);
		Eigen::Vector3f ypr(R2ypr(p_frame_cur_->wRc));
		Eigen::Vector3f t(p_frame_cur_->wTc);
		char text_rotation[128];
		char text_translation[128];
		snprintf(text_rotation, 128, "rotation: %3.2f, %3.2f, %3.2f", ypr.x(), ypr.y(), ypr.z());
		snprintf(text_translation, 128, "translation: %3.2f, %3.2f, %3.2f", t.x(), t.y(), t.z());
		cv::putText(left, text_rotation, cv::Point2i(100, 20), 0, 0.5, cv::Scalar(255, 0, 0));
		cv::putText(left, text_translation, cv::Point2i(100, 50), 0, 0.5, cv::Scalar(255, 0, 0));
		DrawBoard::handle().DrawMatch(ref_image, left, my_matches, p_frame_ref_->left_key_points, p_frame_cur_->left_key_points, false);
		std::cout << "[OrbTracker Track] track ret: " << ret << std::endl;
		//std::cout << "[OrbTracker Track] t: " << (p_frame_cur_->wTc).transpose() << std::endl;
		//DrawBoard::handle().DrawMatch(left, right, p_frame_cur_->match_points, false);
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