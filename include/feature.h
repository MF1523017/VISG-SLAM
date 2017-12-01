#ifndef FEATURE_H
#define FEATURE_H
#include <memory>
#include "type.h"
namespace VISG {

class Feature {
public:
	Feature() = default;
	virtual ~Feature();
	using Ptr = std::shared_ptr<Feature>;
	virtual void Extract(const cv::Mat &img,KeyPoints &keys,cv::Mat & descriptors)=0;
protected:
	cv::Ptr<cv::FeatureDetector> detector_;
};
class OrbFeature:public Feature{
public:
	using Ptr = std::shared_ptr<OrbFeature>;
	OrbFeature();
	virtual ~OrbFeature();
	virtual void Extract(const cv::Mat &img, KeyPoints &keys, cv::Mat & descriptors);

};

}
#endif
