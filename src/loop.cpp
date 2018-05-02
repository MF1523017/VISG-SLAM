#include "loop.h"

namespace VISG {
	bool Loop::MakeDictionary(const std::vector<std::string> &image_names) {
		std::vector<cv::Mat> descriptors;
		descriptors.reserve(image_names.size());

		for (const auto &img_name : image_names) {
			cv::Mat img = cv::imread(img_name);
			std::vector<cv::KeyPoint> key_points;
			cv::Mat descriptor;
			orb_feature_ptr_->Extract(img, key_points, descriptor);
			descriptors.push_back(descriptor);
		}

		vocab_.create(descriptors);
		db_.setVocabulary(vocab_, false, 0);
		std::cout << "[Loop::MakeDictionary]: vocabulary info: " << vocab_ << std::endl;
		return true;
	}

	void Loop::SaveDictionary(const std::string &dictionary) {
		std::cout << "[Loop::SaveDictionary]: saving dictionary... " << vocab_ << std::endl;
		vocab_.save(dictionary);
		std::cout << "done! " << std::endl;
	}

	bool Loop::LoadDictionary(const std::string &dictionary) {
		vocab_.load(dictionary);
		db_.setVocabulary(vocab_, false, 0);
		std::cout << "[Loop::LoadDictionary]: loading dictionary... " << vocab_ << std::endl;
		std::cout << "done! " << std::endl;
		return true;
	}

	void Loop::LoadFeature(const std::vector<std::string> &image_names, std::vector<cv::Mat> &descriptors) {
		descriptors.reserve(image_names.size());

		for (const auto &img_name : image_names) {
			cv::Mat img = cv::imread(img_name);
			std::vector<cv::KeyPoint> key_points;
			cv::Mat descriptor;
			orb_feature_ptr_->Extract(img, key_points, descriptor);
			descriptors.push_back(descriptor);
		}
	}

	void Loop::AddFeatureToDB(const std::vector<cv::Mat> &descriptors) {
		for (const auto & f : descriptors) {
			db_.add(f);
		}
		std::cout << "[Loop::AddFeatureToDB] database info: " << db_ << std::endl;
	}

	void Loop::AddFeatureToDB(const cv::Mat &descriptor) {
		db_.add(descriptor);
	}

	void Loop::ComputeSimilar(const cv::Mat &descriptor, DBoW3::QueryResults &ret) {
		db_.query(descriptor, ret, 3);
		std::cout << "[Loop::ComputeSimilar] query return " << ret << std::endl;
	}


}