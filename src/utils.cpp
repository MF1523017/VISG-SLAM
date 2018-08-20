#include "utils.hpp"

namespace VISG {
	/*
	*brief: convert sl Mat to cv Mat
	*/
	cv::Mat slMat2cvMat(sl::Mat& input) {
		// Mapping between MAT_TYPE and CV_TYPE
		int cv_type = -1;
		switch (input.getDataType()) {
		case sl::MAT_TYPE_32F_C1: cv_type = CV_32FC1; break;
		case sl::MAT_TYPE_32F_C2: cv_type = CV_32FC2; break;
		case sl::MAT_TYPE_32F_C3: cv_type = CV_32FC3; break;
		case sl::MAT_TYPE_32F_C4: cv_type = CV_32FC4; break;
		case sl::MAT_TYPE_8U_C1: cv_type = CV_8UC1; break;
		case sl::MAT_TYPE_8U_C2: cv_type = CV_8UC2; break;
		case sl::MAT_TYPE_8U_C3: cv_type = CV_8UC3; break;
		case sl::MAT_TYPE_8U_C4: cv_type = CV_8UC4; break;
		default: break;
		}

		// Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
		// cv::Mat and sl::Mat will share a single memory structure
		return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM_CPU));
	}
	/*
	*brief: show stereo image left and right
	*/
	void imshow(const std::string &name, const cv::Mat &left, const cv::Mat &right) {
		cv::Size left_size = left.size();
		cv::Size right_size = right.size();

		cv::Size size(left_size.width + right_size.width, left_size.height);
		cv::Mat large(size, left.type());
		large.setTo(0);
		cv::Mat img1 = large(cv::Rect(0, 0, left_size.width, left_size.height));
		cv::Mat img2 = large(cv::Rect(left_size.width, 0, right_size.width, left_size.height));
		left.copyTo(img1);
		right.copyTo(img2);

		cv::imshow(name, large);
	}

	/*
	* brief: convert depth to image to save
	*/

	void depth2Image(const cv::Mat& depth, cv::Mat &depth_image) {
		for (int i = 0; i < depth.rows; ++i) {
			const float *p = depth.ptr<float>(i);
			uchar *q = depth_image.ptr<uchar>(i);
			for (int j = 0; j < depth.cols; ++j) {
				//std::cout << "p[j]: " << p[j] << std::endl;
				if (p[j] > 0 && p[j] < 60) {
					q[j] = static_cast<uchar>(10 * p[j]);
					//std::cout << "p[j]: " << p[j] << std::endl;
				}
			}
		}
	}

	Eigen::Matrix3f Rcv2Eigen(const cv::Mat &R_rhs) {
		cv::Mat R;
		R_rhs.convertTo(R, CV_32F);
		Eigen::Matrix3f REigen;
		REigen << R.at<float>(0, 0), R.at<float>(0, 1), R.at<float>(0, 2),
			R.at<float>(1, 0), R.at<float>(1, 1), R.at<float>(1, 2),
			R.at<float>(2, 0), R.at<float>(2, 1), R.at<float>(2, 2);
		return REigen;
	}

	Eigen::Vector3f Tcv2Eigen(const cv::Mat &t_rhs) {
		cv::Mat t;
		t_rhs.convertTo(t, CV_32F);
		return Eigen::Vector3f(t.at<float>(0, 0), t.at<float>(1, 0), t.at<float>(2, 0));
	}

	Eigen::Vector3f Pcv2Eigen(const cv::Point3f &p) {
		return Eigen::Vector3f(p.x, p.y, p.z);
	}

	cv::Point3f PEigen2cv(const Eigen::Vector3f &p) {
		return cv::Point3f(p.x(), p.y(), p.z());
	}
	cv::Point3f Pdouble2cv(const double *p) {
		return cv::Point3f(p[0], p[1], p[2]);
	}

	Eigen::Vector3f R2ypr(const Eigen::Matrix3f &R) {
		Eigen::Vector3f n = R.col(0);
		Eigen::Vector3f o = R.col(1);
		Eigen::Vector3f a = R.col(2);

		Eigen::Vector3f ypr(3);
		float y = atan2(n(1), n(0));
		float p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
		float r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
		ypr(0) = y;
		ypr(1) = p;
		ypr(2) = r;

		return ypr / M_PI * 180.0;
	}
	void HPose(const Eigen::Matrix3f &R, const Eigen::Vector3f &t, Eigen::Matrix4f &T) {
		T << R(0, 0), R(0, 1), R(0, 2), t.x(),
			R(1, 0), R(1, 1), R(1, 2), t.y(),
			R(2, 0), R(2, 1), R(2, 2), t.z(),
			0, 0, 0, 1;
	}

	void HPose2Rt(const Eigen::Matrix4f &T, Eigen::Matrix3f &R, Eigen::Vector3f &t) {
		R << T(0,0),T(0,1),T(0,2),
			T(1, 0), T(1, 1), T(1, 2),
			T(2, 0), T(2, 1), T(2, 2);
		t << T(0, 3), T(1, 3), T(2, 3);
	}

	void loadImage(const std::string & file_dir, std::vector<std::string> &images) {
		//const std::string data_file(file_dir + "\\cam0\\data.csv");
		const std::string data_file(file_dir + "\\cam0\\data.csv");
		std::ifstream read_images(data_file);
		std::string line;
		while (read_images && !read_images.eof()) {
			std::getline(read_images, line);
			if ('#' == line[0])
				continue;
			size_t pos = line.find(",");
			auto image_name = line.substr(pos + 1, line.size() - pos - 2);
			if (image_name.empty())
				break;
			//std::cout << "[image_name]: " << image_name << std::endl;
			images.push_back(image_name);
		}
		read_images.close();
	}

	void SaveXYZ(const std::string & file_name, const cv::Mat& mat, const cv::Mat &img) {
		const double max_z = 20;
		std::ofstream of(file_name);
		for (int y = 0; y < mat.rows; y++){
			for (int x = 0; x < mat.cols; x++){
				cv::Vec3f point = mat.at<cv::Vec3f>(y, x);
				double dis2 = point.dot(point);
				cv::Vec3b color = img.at<cv::Vec3b>(y, x);
				if (dis2 > 0.09||fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
				of <<"v " <<point[0] << " " << point[1] << " " << point[2] << " "<< 
					(int)color[2] << " " << (int)color[1] << " " << (int)color[0] <<std::endl;;
			}
		}
		of.close();
	}

	void SaveT(const std::string & file_name, const std::vector<Eigen::Vector3f> & positions) {
		std::ofstream of(file_name);
		for (const auto & p : positions) {
			of << p.x() <<" "<< p.y() <<" "<< p.z() << std::endl;
		}
		of.close();
	}
}