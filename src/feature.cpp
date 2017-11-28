#include "feature.h"

namespace VISG {
Feature::~Feature() {
	key_points_.clear();
	descriptors_.release();
}
OrbFeature::OrbFeature() {
	detector_ = cv::ORB::create(1000,1.2,10,50);
}

void OrbFeature::Extract(const cv::Mat &img) {
	if (img.empty())
		assert(false);
	key_points_.clear();//clear old key_points
	descriptors_.release();// clear old descriptors
	detector_->detect(img, key_points_);
	detector_->compute(img, key_points_, descriptors_);
}

OrbFeature::~OrbFeature(){
	key_points_.clear();
	descriptors_.release();
}
}