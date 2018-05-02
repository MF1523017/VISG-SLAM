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
	virtual int GetPose(Eigen::Matrix3f& R, Eigen::Vector3f &t) const = 0;
	virtual void GetMapPoints(std::vector<Eigen::Vector3f> &map_points,std::vector<Eigen::Vector3i> &colors)= 0;
	TrackingState GetTrackingState()const { return state_; }
	virtual void GetKeyFramePositions(std::vector<Eigen::Vector3f> &positions)const=0;
protected:
	TrackingState state_;
	bool is_only_tracking_;

};

}


#endif // tracker_interface