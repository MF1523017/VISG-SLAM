#ifndef TRACKER_INTERFACE_H
#define TRACKER_INTERFACE_H
#include "common.h"
namespace VISG {
class TrackerInterface {
public:
	using Ptr = std::shared_ptr<TrackerInterface>;
	enum TrackingState
	{
		INIT = 0,
		TRACKING = 1,
		LOST = 2,
	};
	TrackerInterface() :state_(INIT), is_only_tracking_(true) {};
	virtual ~TrackerInterface() {}
	virtual bool Init(cv::Mat &left, cv::Mat &right)=0;
	virtual void operator()(cv::Mat &left, cv::Mat &right) = 0;
	virtual bool Track(cv::Mat &left, cv::Mat &right) = 0;
	virtual void Reboot() = 0;
	virtual int GetPose(cv::Mat&R, cv::Mat &t) const = 0;
	virtual std::vector<cv::Point3f> GetMapPoints()const = 0;
	TrackingState GetTrackingState()const { return state_; }
protected:
	TrackingState state_;
	bool is_only_tracking_;

};

}


#endif // tracker_interface