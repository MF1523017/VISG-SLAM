#include "draw_board.h"
#include "utils.hpp"
#include "delaunay.h"

namespace VISG {
void DrawBoard::DrawFeatures(cv::Mat &img, const KeyPoints &key_points, bool showing){
	cv::Mat img_key_points;
	cv::drawKeypoints(img, key_points, img_key_points, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
	cv::imshow("black_board", img_key_points);
	if(showing)
		while (cv::waitKey() == 27);
}
void DrawBoard::DrawMatch(cv::Mat &left, cv::Mat &right, MatchPoints &match_points) {
	cv::Size left_size = left.size();
	cv::Size right_size = right.size();

	cv::Size size(left_size.width + right_size.width, left_size.height);
	cv::Mat large(size, left.type());
	large.setTo(0);
	cv::Mat img1 = large(cv::Rect(0, 0, left_size.width, left_size.height));
	cv::Mat img2 = large(cv::Rect(left_size.width, 0, right_size.width, left_size.height));
	left.copyTo(img1);
	right.copyTo(img2);
	for (size_t i = 0; i < match_points.size(); ++i) {
		cv::Point2f lp(match_points[i].x(), match_points[i].y());
		cv::Point2f rp(match_points[i].z() + left_size.width, match_points[i].y());
		cv::circle(large, lp, 10, cv::Scalar(255, 0, 0));
		cv::circle(large, rp, 10, cv::Scalar(255, 0, 0));
		cv::line(large, lp, rp, cv::Scalar(0, 0, 255));
		cv::putText(large, std::to_string(i), lp, 0, 0.5, cv::Scalar(255, 0, 0));
		cv::putText(large, std::to_string(i), rp, 0, 0.5, cv::Scalar(255, 0, 0));
	}
	ShowBlackBoard(large);
}

void DrawBoard::DrawMatch(cv::Mat &left, cv::Mat &right, MyMatches &my_matches, const KeyPoints& key_points1, const KeyPoints& key_points2) {
	cv::Size left_size = left.size();
	cv::Size right_size = right.size();

	cv::Size size(left_size.width + right_size.width, left_size.height);
	cv::Mat large(size, left.type());
	large.setTo(0);
	cv::Mat img1 = large(cv::Rect(0, 0, left_size.width, left_size.height));
	cv::Mat img2 = large(cv::Rect(left_size.width, 0, right_size.width, left_size.height));
	left.copyTo(img1);
	right.copyTo(img2);
	for (size_t i = 0; i < my_matches.size(); ++i) {
		const int & queryIdx = my_matches[i].first;
		const int & trainIdx = my_matches[i].second;
		cv::Point2f lp(key_points1[queryIdx].pt);
		cv::Point2f rp(key_points2[trainIdx].pt.x + left_size.width, key_points2[trainIdx].pt.y);
		cv::circle(large, lp, 4, cv::Scalar(125, 0, 0));
		cv::circle(large, rp, 4, cv::Scalar(125, 0, 0));
		cv::line(large, lp, rp, cv::Scalar(0, 0, 100));
		//cv::putText(large, std::to_string(i), lp, 0, 0.5, cv::Scalar(255, 0, 0));
		//cv::putText(large, std::to_string(i), rp, 0, 0.5, cv::Scalar(255, 0, 0));
	}
	ShowBlackBoard(large);
}
void DrawBoard::DrawPose(cv::Mat &left,const Eigen::Matrix3f & R, const Eigen::Vector3f &t,bool status) {

	Eigen::Vector3f ypr(R2ypr(R));
	char text_rotation[128];
	char text_translation[128];
	char test_status[128];
	snprintf(text_rotation, 128, "rotation: %3.3f, %3.3f, %3.3f", ypr.x(), ypr.y(), ypr.z());
	snprintf(text_translation, 128, "translation: %3.3f, %3.3f, %3.3f", t.x(), t.y(), t.z());
	snprintf(test_status, 128, "status: %d", status);
	cv::putText(left, text_rotation, cv::Point2i(100, 20), 0, 0.5, cv::Scalar(255, 0, 255));
	cv::putText(left, text_translation, cv::Point2i(100, 50), 0, 0.5, cv::Scalar(255, 0, 255));
	cv::putText(left, test_status, cv::Point2i(100, 80), 0, 0.5, cv::Scalar(255, 0, 255));
}

void DrawBoard::DrawCorner(cv::Mat &img,const cv::Size &pattern_size,const std::vector<cv::Point2f> & corners,bool patternfound) {
	cv::drawChessboardCorners(img, pattern_size, corners, patternfound);
	cv::imshow("black_board", img);
}

void DrawBoard::DrawProjectError(cv::Mat &img, const KeyPoints &key_points, const MyMatches &matches, const std::vector<cv::Point2f> &pro_points) {

	for (size_t i = 0; i < matches.size(); ++i) {
		size_t trainIdx = matches[i].second;
		cv::circle(img, pro_points[i], 4, cv::Scalar(255, 0, 0));
		cv::line(img, key_points[trainIdx].pt, pro_points[i], cv::Scalar(255, 0, 0),2);
	}
}


Eigen::Vector3f DrawBoard::FindGround(const std::vector<Eigen::Vector3f> &point_cloud, std::vector<Eigen::Vector3f> &inlier_points) {
	int height_range[30];
	double height_sum[30];
	std::vector<std::vector<Eigen::Vector3f>> points_clusters;
	points_clusters.resize(30);
	for (int i = 0; i < 30; i++)
	{
		height_range[i] = 0;
		height_sum[i] = 0;
	}
	for (unsigned int i = 0; i < point_cloud.size(); i++)
	{
		double y = point_cloud[i].y();
		int index = (y + 2.0) / 0.1;
		if (0 <= index && index < 30)
		{
			height_range[index]++;
			height_sum[index] += y;
			points_clusters[index].push_back(point_cloud[i]);
		}
	}
	int max_num = 0;
	int max_index = -1;
	for (int i = 1; i < 29; i++)
	{
		if (max_num < height_range[i])
		{
			max_num = height_range[i];
			max_index = i;
		}
	}
	if (max_index == -1)
		return Eigen::Vector3f(0, 0, 0);
	else
	{
		inlier_points = points_clusters[max_index];
		Eigen::Vector3f tmp_p;
		tmp_p.setZero();
		for (int i = 0; i< inlier_points.size(); i++)
		{
			tmp_p += inlier_points[i];
		}
		return tmp_p / inlier_points.size();
	}
}

void DrawBoard::DrawGround(cv::Mat &result, std::vector<Eigen::Vector3f> &point_cloud, const Eigen::Matrix3f wRc, const Eigen::Vector3f &wtc) {
	std::vector<Vec2f_> points;
	for (unsigned int i = 0; i < point_cloud.size(); i++)
	{
		Eigen::Vector3f Pc = wRc.transpose() *(point_cloud[i] - wtc);
		cv::Point2f pts;
		pts.x = Common::Fx * Pc.x() / Pc.z() + Common::Cx;
		pts.y = Common::Fy * Pc.y() / Pc.z() + Common::Cy;
		//	std::cout << "[point2f]: x " << pts.x << " y: " << pts.y << std::endl;
		points.push_back(Vec2f_(pts.x, pts.y));

	}
	Delaunay triangulation;
	std::vector<Triangle> triangles = triangulation.triangulate(points);
	//  std::cout << triangles.size() << " triangles generated\n";
	std::vector<Edge> edges = triangulation.getEdges();

	for (auto e = begin(edges); e != end(edges); e++) {
		cv::Point2f pts, pts2;
		pts.x = (*e).p1.x;
		pts.y = (*e).p1.y;
		pts2.x = (*e).p2.x;
		pts2.y = (*e).p2.y;
		cv::line(result, pts, pts2, cvScalar(255, 0, 0), 1, 8, 0);
	}
}

void DrawBoard::DrawBox(cv::Mat &result, const Box &box, const Eigen::Matrix3f wRc, const Eigen::Vector3f &wtc) {
	std::vector<Eigen::Vector3f> boxConers;
	boxConers.push_back(box.ori);
	boxConers.push_back(box.cox);
	boxConers.push_back(box.coy);
	boxConers.push_back(box.cox + box.coy - box.ori);

	boxConers.push_back(box.coz);
	boxConers.push_back(box.cox + box.coz - box.ori);
	boxConers.push_back(box.coy + box.coz - box.ori);
	boxConers.push_back(box.cox + box.coy + box.coz - 2.0*box.ori);

	std::vector<cv::Point2f> boxImage;
	Eigen::Vector3f Pc;
	std::vector<float> depth_of_coner;
	for (auto it : boxConers)
	{

		Pc = wRc.transpose() * (it - wtc);
		if (Pc.z()<0)
			return;
		cv::Point2f pts;

		pts.x = Common::Fx * Pc.x() / Pc.z() + Common::Cx;
		pts.y = Common::Fy * Pc.y() / Pc.z() + Common::Cy;

		depth_of_coner.push_back(Pc.norm());
		boxImage.push_back(pts);
	}



	//draw color
	cv::Point* p = new cv::Point[8];
	p[0] = boxImage[0];
	p[1] = boxImage[1];
	p[2] = boxImage[2];
	p[3] = boxImage[3];
	p[4] = boxImage[4];
	p[5] = boxImage[5];
	p[6] = boxImage[6];
	p[7] = boxImage[7];

	int npts[1] = { 4 };
	float min_depth = 10;
	int min_index = 5;
	for (int i = 0; i< depth_of_coner.size(); i++)
	{
		if (depth_of_coner[i] < min_depth)
		{
			min_depth = depth_of_coner[i];
			min_index = i;
		}
	}

	cv::Point plain[1][4];
	const cv::Point* ppt[1] = { plain[0] };
	//first draw large depth plane
	int point_group[8][12] = { { 0,1,5,4, 0,4,6,2, 0,1,3,2 },
	{ 0,1,5,4, 1,5,7,3, 0,1,3,2 },
	{ 2,3,7,6, 0,4,6,2, 0,1,3,2 },
	{ 2,3,7,6, 1,5,7,3, 0,1,3,2 },
	{ 0,1,5,4, 0,4,6,2, 4,5,7,6 },
	{ 0,1,5,4, 1,5,7,3, 4,5,7,6 },
	{ 2,3,7,6, 0,4,6,2, 4,5,7,6 },
	{ 2,3,7,6, 1,5,7,3, 4,5,7,6 } };
	float aver_depth = 0;

	plain[0][0] = p[point_group[min_index][4]];
	plain[0][1] = p[point_group[min_index][5]];
	plain[0][2] = p[point_group[min_index][6]];
	plain[0][3] = p[point_group[min_index][7]];
	cv::fillPoly(result, ppt, npts, 1, cv::Scalar(0, 200, 0));

	plain[0][0] = p[point_group[min_index][0]];
	plain[0][1] = p[point_group[min_index][1]];
	plain[0][2] = p[point_group[min_index][2]];
	plain[0][3] = p[point_group[min_index][3]];
	cv::fillPoly(result, ppt, npts, 1, cv::Scalar(200, 0, 0));

	if (depth_of_coner[point_group[min_index][2]] + depth_of_coner[point_group[min_index][3]] >
		depth_of_coner[point_group[min_index][5]] + depth_of_coner[point_group[min_index][6]])
	{
		plain[0][0] = p[point_group[min_index][4]];
		plain[0][1] = p[point_group[min_index][5]];
		plain[0][2] = p[point_group[min_index][6]];
		plain[0][3] = p[point_group[min_index][7]];
		cv::fillPoly(result, ppt, npts, 1, cv::Scalar(0, 200, 0));

	}
	plain[0][0] = p[point_group[min_index][8]];
	plain[0][1] = p[point_group[min_index][9]];
	plain[0][2] = p[point_group[min_index][10]];
	plain[0][3] = p[point_group[min_index][11]];
	cv::fillPoly(result, ppt, npts, 1, cv::Scalar(0, 0, 200));
}

Eigen::Vector4f DrawBoard::CreatPlane(const Eigen::Vector3f &p1, const Eigen::Vector3f &p2, const Eigen::Vector3f &p3) {
	Eigen::Vector3f arrow1 = p2 - p1;
	Eigen::Vector3f arrow2 = p3 - p1;
	Eigen::Vector3f normal;
	normal = arrow1.cross(arrow2);
	return Eigen::Vector4f(normal.x(), normal.y(), normal.z(),
		-normal.x() * p1.x() - normal.y() * p1.y() - normal.z() * p1.z());
}


Eigen::Vector4f DrawBoard::FindPlane(std::vector<Eigen::Vector3f> &point_cloud) {
	int K = 2000, k = 1;    //max iterate num and current iterate num
	float sigma = 0.01;
	int pretotal = 0;
	Eigen::Vector4f bestplane;
	bestplane << 0, 0, 0, 0;

	while (pretotal<point_cloud.size() / 2 && k < K)
	{
		int index1 = rand() % point_cloud.size();
		int index2 = rand() % point_cloud.size();
		int index3 = rand() % point_cloud.size();
		while (index2 == index1)
			index2 = rand() % point_cloud.size();
		while (index3 == index2 || index3 == index1)
			index3 = rand() % point_cloud.size();
		Eigen::Vector3f point1, point2, point3;
		point1 = point_cloud[index1];
		point2 = point_cloud[index2];
		point3 = point_cloud[index3];

		Eigen::Vector4f plane = CreatPlane(point1, point2, point3);
		int inPlaneNum = 0;
		Eigen::Vector3f point_sum;
		point_sum.setZero();
		for (int i = 0; i< point_cloud.size(); i++)
		{
			if (i == index1 || i == index2 || i == index3)
				continue;
			Eigen::Vector3f tmp;
			tmp << plane(0), plane(1), plane(2);
			float dist = fabs(tmp.dot(point_cloud[i]) + plane(3)) / (tmp).norm();
			if (dist < sigma)
			{
				inPlaneNum++;
				point_sum += point_cloud[i];
			}
		}
		if (inPlaneNum > pretotal)
		{
			pretotal = inPlaneNum;
			bestplane = plane;
		}
		k++;
	}
	std::cout << "Plane: " << pretotal << " iter:" << k << std::endl;
	return bestplane;
}

void DrawBoard::DrawAR(cv::Mat &result, const std::vector<Eigen::Vector3f> &point_cloud, const Eigen::Matrix3f wRc, const Eigen::Vector3f &wtc) {

	Eigen::Vector3f ground_plane_point;
	std::vector<Eigen::Vector3f> point_inlier;
	//std::cout << "[drawAR]: points size: " << point_cloud.size() << std::endl;
	ground_plane_point = FindGround(point_cloud, point_inlier);
	//printf("[drawAR]:Ground inlier size %d\n", int(point_inlier.size()) );


	///draw ground area
	if (point_inlier.size()>10)
		DrawGround(result, point_inlier, wRc, wtc);

	//draw existing boxes
	for (unsigned int i = 0; i< boxes_.size(); i++){
		DrawBox(result, boxes_[i], wRc, wtc);
	}
	//add new box
	//std::cout << "[drawAR] tapFlag: " << tapFlag << " point_inlier size: " << point_inlier.size() << std::endl;
	if (is_tap && point_inlier.size()>10)
	{

		Eigen::Matrix3f RIC;
		RIC << 1, 0, 0,
			0, 1, 0,
			0, 0, 1;
		float xx = tap_x;
		float yy = tap_y;
		Eigen::Vector3f pc;
		Eigen::Vector2f box_center, center_input;
		center_input << xx, yy;
		/*std::cout << "[drawAR]: ground_plane_point: x: " << groundPlanePoint.x() << ", y: " <<
		groundPlanePoint.y() << ", z: " << groundPlanePoint.z() << std::endl;*/
		pc = wRc.transpose()* (ground_plane_point - wtc);
		box_center.x() = Common::Fx * pc.x() / pc.z() + Common::Cx;
		box_center.y() = Common::Fy * pc.y() / pc.z() + Common::Cy;
		if ((box_center - center_input).norm()<200)
		{
			Eigen::Vector3f ori, cox, coy, coz;
			Eigen::Vector3f linex, liney, linez;
			float lengthc;

			lengthc = (wtc - ground_plane_point).norm()*0.15;
			linex = Eigen::Vector3f(1, 0, 0);
			liney = Eigen::Vector3f(0, 1, 0);
			linez = Eigen::Vector3f(0, 0, 1);
			ori = ground_plane_point - (linex*lengthc + liney*lengthc) / 2.0;

			cox = ori + linex*lengthc;
			coy = ori + liney*lengthc;
			coz = ori + linez*lengthc;

			
			Box box(ground_plane_point, lengthc);

			box.ori = ori;
			box.cox = cox;
			box.coy = coy;
			box.coz = coz;
			box.lix = linex;
			box.liy = liney;
			box.liz = linez;
			box.initPlane = FindPlane(point_inlier);
			boxes_.push_back(box);
			DrawBox(result, box,wRc,wtc);
		}

	}
	is_tap = false;
}

void OnMouse(int event, int x, int y, int flags, void*ustc) {
	if (event == CV_EVENT_LBUTTONDOWN) {
		DrawBoard::handle().tap_x = x;
		DrawBoard::handle().tap_y = y;
		DrawBoard::handle().is_tap = true;
	}
}


}