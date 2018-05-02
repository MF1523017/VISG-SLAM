#ifndef ORB_TRACKER_H
#define ORB_TRACKER_H
#include "tracker_interface.h"
#include "frame.h"
#include "keyframe.h"
#include "loop.h"


namespace VISG {
class OrbTracker:public TrackerInterface {
public:
	OrbTracker();
	OrbTracker(const std::string & dict);
	virtual ~OrbTracker() {
		std::cout << "[~OrbTracker] key_frames size: " << key_frames_.size() << std::endl;
		for (const auto &kf : key_frames_) {
			std::cout << "[~OrbTracker] key_frame id: " << kf->id() << std::endl;
		}
	}
	virtual bool Init(cv::Mat &left, cv::Mat &right);
	virtual void operator()(cv::Mat &left, cv::Mat &right);
	virtual bool Track(cv::Mat &left, cv::Mat &right);
	virtual void Reboot();
	virtual int GetPose(Eigen::Matrix3f& R, Eigen::Vector3f &t) const;
	virtual void GetMapPoints(std::vector<Eigen::Vector3f> &map_points, std::vector<Eigen::Vector3i> &colors);
private:
	bool IsKeyFrame()const;
	int IsLoopClosing()const;
private:
	Frame::Ptr p_frame_cur_;
	Frame::Ptr p_frame_last_;
	Frame::Ptr p_frame_ref_;
	cv::Mat ref_image;
	std::vector<Frame::Ptr> local_frames_;
	std::vector<Frame::Ptr> key_frames_;
	Loop::Ptr loop_closing_;
	size_t motion_counter_;
	size_t frame_id_; 
};
}
#endif // orb_tracker.h
