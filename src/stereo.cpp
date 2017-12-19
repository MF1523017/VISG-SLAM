#include "stereo.h"
namespace VISG {
	void Stereo::Compute(cv::Mat &left, cv::Mat &right, cv::Mat &disp,cv::Mat &points3) {
		///if (left.type() == CV_8UC3)
		cv::Mat R1, R2, P1, P2, Q,D1,D2;
		cv::Mat cam1, cam2, r, t;
		Common::K.convertTo(cam1, CV_64F);
		Common::K.convertTo(cam2, CV_64F);
		Common::lRr.convertTo(r, CV_64F);
		Common::ltr.convertTo(t, CV_64F);

		cv::stereoRectify(cam1, D1, cam2, D2, left.size(), r, t, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, -1, left.size());
		std::cout << "[Stereo::Compute] R1:\n" << R1 << "\nR2:\n" << R2 << "\nQ:\n" << Q << std::endl;
		int num_of_disp = 32;// ((left.size().width / 8 + 15) & -16);// must be divisible by 16
		std::cout << "[Stereo::Compute] num_of_disp: " << num_of_disp << std::endl;
		p_sm->setPreFilterCap(63);//clips its value by [-preFilterCap, preFilterCap]
		int sgbm_win_size = 3;
		p_sm->setBlockSize(sgbm_win_size);//odd number \>=1 3-11
		int cn = left.channels();

		p_sm->setP1(8 * cn*sgbm_win_size*sgbm_win_size);//The first parameter controlling the disparity smoothness
		p_sm->setP2(32 * cn*sgbm_win_size*sgbm_win_size);
		p_sm->setMinDisparity(0);//Normally, it is zero but sometimes
								//rectification algorithms can shift images, so this parameter needs to be adjusted accordingly.
		p_sm->setNumDisparities(num_of_disp);
		p_sm->setSpeckleWindowSize(100);//Maximum size of smooth disparity regions to consider their noise speckles
										//and invalidate. 50-200
		p_sm->setUniquenessRatio(10);/* Margin in percentage by which the best(minimum) computed cost function
									value should "win" the second best value to consider the found match correct.Normally, a value
									within the 5 - 15 range is good enough*/
		p_sm->setSpeckleRange(32);
		p_sm->setDisp12MaxDiff(1);
		p_sm->setMode(cv::StereoSGBM::MODE_SGBM);
		p_sm->compute(left, right, disp);
		//float lastPixel = 0.5;
		//float minDisparity = 20;// algorithm parameters that can be modified
		//for (int i = 0; i < disp.rows; i++)
		//{
		//	for (int j = num_of_disp; j < disp.cols; j++)
		//	{
		//		if (disp.at<float>(i, j) <= minDisparity) disp.at<float>(i, j) = lastPixel;
		//		else lastPixel = disp.at<float>(i, j);
		//	}
		//}
		
		cv::Mat disp8;
		disp.convertTo(disp8, CV_8U, 255 / (num_of_disp*16.));
		cv::reprojectImageTo3D(disp, points3, Q, true);
		
		cv::namedWindow("left", 1);
		cv::imshow("left", left);
		cv::namedWindow("right", 1);
		cv::imshow("right", right);
		cv::namedWindow("disparity", 0);
		cv::imshow("disparity", disp8);
		cv::waitKey();
	}
}