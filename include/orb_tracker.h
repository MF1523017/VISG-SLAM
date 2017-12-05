#ifndef ORB_TRACKER_H
#define ORB_TRACKER_H
#include "tracker_interface.h"
#include "frame.h"
#include "keyframe.h"
namespace VISG {
class OrbTracker:public TrackerInterface {
public:
	OrbTracker();
	virtual ~OrbTracker() {}
	virtual bool Init(cv::Mat &left, cv::Mat &right);
	virtual void operator()(cv::Mat &left, cv::Mat &right);
	virtual bool Track(cv::Mat &left, cv::Mat &right);
	virtual void Reboot();
	virtual int GetPose(cv::Mat&R, cv::Mat &t) const;
	virtual std::vector<cv::Point3f> GetMapPoints()const;
private:
	Frame::Ptr p_frame_cur_;
	Frame::Ptr p_frame_last_;
	Frame::Ptr p_frame_ref_;
	cv::Mat ref_image;
	std::vector<KeyFrame::Ptr> key_frames;
};
}
#endif // orb_tracker.h
