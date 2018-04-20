#ifndef FRAME_H
#define FRAME_H
#include "feature.h"
#include "map_point.h"
#include <vector>
namespace VISG {
class Frame {
public:
	using Ptr = std::shared_ptr<Frame>;
	Frame() = default;
	Frame(size_t id):p_orb_left_(new OrbFeature), p_orb_right_(new OrbFeature) ,id_(id){
		wRc << 1, 0, 0,
			0, 1, 0,
			0, 0, 1;
		wtc<< 0,0,0;
		wTc<<1,0,0,0,
			0,1,0,0,
			0,0,1,0,
			0,0,0,1;
		rRc = wRc;
		rtc = wtc;
	}
	void SetPose(const Eigen::Matrix3f &R, const Eigen::Vector3f &t);
	void ExtractFeatures(const cv::Mat&left, const cv::Mat&right);
	void ExtractORB(const cv::Mat&img, bool is_left);
	
	size_t StereoMatch(const cv::Mat &left);
	bool RefTrack2D2D(Frame::Ptr p_frame_ref, MyMatches&inliers_matches);
	bool RefTrack2D3D(Frame::Ptr p_frame_ref, MyMatches &inliers_matches);
	bool RefTrack3D3D(Frame::Ptr p_frame_ref, MyMatches &inliers_matches);
	void MotionTrack(Frame::Ptr p_frame_ref);
	bool FetchMatches(Frame::Ptr p_frame_ref, MyMatches &inliers_matches);
	bool IsKeyFrame(MyMatches &matches);
	void GetwMapPoints(std::vector<Eigen::Vector3f> &valid_map_points);
	~Frame();
	const size_t id()const { return id_; }
public:
	cv::Mat left_descriptors, right_descriptors;
	KeyPoints left_key_points, right_key_points;
	MatchPoints match_points;// (ul,vl,ur)
	std::vector<MapPoint::Ptr> map_points;
	Eigen::Matrix3f wRc;//rotation camera to world frame
	Eigen::Vector3f wtc;//translation
	Eigen::Matrix3f rRc;// camera to reference frame
	Eigen::Vector3f rtc;
	Eigen::Matrix4f wTc;// pose
	std::vector<bool> inliers;

private:
	void Reset();
	bool RecoverPose(Frame::Ptr p_frame_ref,MyMatches &inliers_matches,cv::Mat &R,cv::Mat &t);
	bool RecoverPoseWithcvPnp(Frame::Ptr p_frame_ref, MyMatches &inliers_matches, cv::Mat &R, cv::Mat &t);
	bool RecoverPoseWithPnpSolver(Frame::Ptr p_frame_ref, MyMatches &inliers_matches, cv::Mat &R, cv::Mat &t);
	bool RecoverPoseWithStereoMatchesPnp(Frame::Ptr p_frame_ref, MyMatches &inliers_matches, cv::Mat &R, cv::Mat &t);
	Feature::Ptr p_orb_left_;
	Feature::Ptr p_orb_right_;
	size_t id_;

};

}

#endif //frame.h
