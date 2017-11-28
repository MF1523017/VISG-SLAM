#include "visg_slam.h"
#include "draw_board.h"
namespace VISG {
	VisgSlam::VisgSlam() :p_cur_frame(new Frame), p_last_frame(new Frame) {
		zed_.Open();
	}
	void VisgSlam::Run(cv::Mat left, cv::Mat right) {
		while (true) {
			// Grab an image
			if (zed_.Grab(left_, right_)) {
				cv::cvtColor(left_, left_, CV_RGBA2GRAY);
				cv::cvtColor(right_, right_, CV_RGBA2GRAY);
				p_cur_frame->ExtractFeature(left_, right_);
				DrawBoard::handle().DrawFeatures(left_, p_cur_frame->left_key_points, false);
				std::cout << "fps: " << zed_.GetFPS() << std::endl;
				int key = cv::waitKey(5);
				if (key == 27)
					break;

			}
		}
	}
}