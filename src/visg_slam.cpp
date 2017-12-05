#include "visg_slam.h"
#include "draw_board.h"
namespace VISG {
	VisgSlam::VisgSlam():tracker_(new OrbTracker) {
		zed_.Open();
		Common::Height = zed_.cam_info.left_cam.image_size.height;
		Common::Weight = zed_.cam_info.left_cam.image_size.width;
		Common::Fx = zed_.cam_info.left_cam.fx;
		Common::Fy = zed_.cam_info.left_cam.fy;
		Common::Cx = zed_.cam_info.left_cam.cx;
		Common::Cy = zed_.cam_info.left_cam.cy;
		Common::K = (cv::Mat_<float>(3, 3) << Common::Fx, 0, Common::Cx,
			0, Common::Fy, Common::Cy,
			0, 0, 1);

		Common::lRr = zed_.cam_info.extrinsic.R;
		Common::ltr = zed_.cam_info.extrinsic.t;
		Common::FxInv = 1.0 / Common::Fx;
		Common::FyInv = 1.0 / Common::Fy;
	}
	void VisgSlam::Run(cv::Mat left, cv::Mat right) {
		while (true) {
			// Grab an image
			if (zed_.Grab(left_, right_)) {
				cv::cvtColor(left_, left, CV_RGBA2GRAY);
				cv::cvtColor(right_, right, CV_RGBA2GRAY);
				(*tracker_)(left, right);
				std::cout << "fps: " << zed_.GetFPS() << std::endl;
				int key = cv::waitKey(5);
				if (key == 27)
					break;
			/*	std::cout << "left image size: " << zed_.cam_info.left_cam.image_size <<
					" right image size: " << zed_.cam_info.right_cam.image_size << std::endl;*/
			}
		}
	}
}