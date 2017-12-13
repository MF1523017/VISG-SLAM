#include "feature.h"

namespace VISG {
Feature::~Feature() {
}
OrbFeature::OrbFeature() {
	detector_ = cv::ORB::create(Common::FeaturesNum,Common::ScaleFactor,Common::Levels);
}

void OrbFeature::Extract(const cv::Mat &img, KeyPoints &keys, cv::Mat & descriptors) {
	if (img.empty())
		assert(false);
	keys.clear();//clear old key_points
	descriptors.release();// clear old descriptors
	detector_->detect(img, keys);
	detector_->compute(img, keys, descriptors);
}

OrbFeature::~OrbFeature(){
}
}