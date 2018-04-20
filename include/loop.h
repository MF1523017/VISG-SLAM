#ifndef LOOP_H
#define LOOP_H
#include "feature.h"
#include <iostream>
#include <vector>
#include <string>
#include <DBoW3.h>

namespace VISG {
	class Loop {
	public:
		Loop() :orb_feature_ptr_(new OrbFeature) {}
		bool MakeDictionary(const std::vector<std::string> &image_names);
		void SaveDictionary(const std::string &dictionary);
		~Loop() {
			vocab_.clear();
			db_.clear();
		}
		bool LoadDictionary(const std::string &dictionary);
		void LoadFeature(const std::vector<std::string> &image_names, std::vector<cv::Mat> &descriptors);
		void AddFeatureToDB(const std::vector<cv::Mat> &descriptors);
		void ComputeSimilar(const cv::Mat &descriptor);
	private:
		OrbFeature::Ptr orb_feature_ptr_;
		DBoW3::Vocabulary vocab_;
		DBoW3::Database db_;
	};
}

#endif
