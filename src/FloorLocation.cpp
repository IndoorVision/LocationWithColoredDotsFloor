// CodingPointDectect.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <iomanip>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/core.hpp> 

using namespace std;
using namespace cv;

/***自定义比较函数(按类别排序)***/
bool CompByPID(const Vec6i l1, const Vec6i l2)
{
	return l1[4] < l2[4];  //from smaller to larger 
}

/***自定义比较函数(按角度排序)***/
bool CompByAngle(const Vec4i a1, const Vec4i a2)
{
	return a1[3] < a2[3];  //from smaller to larger 
}

/***计算两点间距离***/
double GetDistanceP2P(Point2i p1, Point2i p2)
{
	return sqrt(pow((p1.y - p2.y), 2) + pow((p1.x - p2.x), 2));
}

/***计算两向量夹角***/
float GetAngleOfTwoVector(Point2i pt1, Point2i pt2, Point2i c)
{
	//以pt1为基准，c为向量起点
	float theta = atan2(pt1.x - c.x, pt1.y - c.y) - atan2(pt2.x - c.x, pt2.y - c.y);
	if (theta > CV_PI)
		theta -= 2 * CV_PI;
	if (theta < -CV_PI)
		theta += 2 * CV_PI;
	theta *= 180.0 / CV_PI;
	if (theta < 0)
	{
		theta += 360;
	}
	return theta;
}

/***颜色判断(HSV)***/
int GetColorCode(Mat img, Point2i pt)
{
	int colorCode;
	int H = img.at<Vec3b>(pt)[0];
	int S = img.at<Vec3b>(pt)[1];
	int V = img.at<Vec3b>(pt)[2];

	//红色
	if (((H >= 0 && H <= 10) || (H >= 156 && H <= 180)) && S >= 100 && S <= 255 && V >= 46 && V <= 255)
	{
		colorCode = 1;
	}
	//绿色
	else if (H >= 35 && H <= 80 && S >= 100 && S <= 255 && V >= 46 && V <= 255)
	{
		colorCode = 2;
	}
	//蓝色
	else if (H >= 100 && H <= 124 && S >= 100 && S <= 255 && V >= 46 && V <= 255)
	{
		colorCode = 3;
	}
	//黄色
	else if (H >= 23 && H <= 34 && S >= 100 && S <= 255 && V >= 46 && V <= 255)
	{
		colorCode = 4;
	}
	////灰黑色
	//else if (H <= 23 && S <= 100 && V >= 46 && V <= 255)
	//{
	//	colorCode = 0;
	//}
	else
	{
		colorCode = -1;
	}
	return colorCode;
}

/***计算直线延长线端点***/
void LineExtend(Mat img, Vec4i line, vector<Point2i> &edgePoints)
{
	edgePoints.push_back(Point2i(0, 0));
	edgePoints.push_back(Point2i(0, 0));
	vector<Point2i> tempPoints;
	double d1, d2;

	int x1 = 0;
	int y1 = 0;
	int x2 = img.cols - 1;
	int y2 = img.rows - 1;

	double k = (double)(line[3] - line[1]) / (double)(line[2] - line[0] + 0.00001); //直线斜率
	double b = line[1] - k * line[0]; //直线截距

	int y1_ = (int)(k * x1 + b);
	int y2_ = (int)(k * x2 + b);
	int x1_ = (int)((y1 - b) / k);
	int x2_ = (int)((y2 - b) / k);

	if (y1_ >= 0 && y1_ <= img.rows - 1)
	{
		tempPoints.push_back(Point2i(x1, y1_));
	}

	if (x1_ >= 0 && x1_ <= img.cols - 1)
	{
		tempPoints.push_back(Point2i(x1_, y1));
	}

	if (y2_ >= 0 && y2_ <= img.rows - 1)
	{
		tempPoints.push_back(Point2i(x2, y2_));
	}

	if (x2_ >= 0 && x2_ <= img.cols - 1)
	{
		tempPoints.push_back(Point2i(x2_, y2));
	}

	if ((y1_ < 0 || y1_ > img.rows - 1) && (x1_ < 0 || x1_ > img.cols - 1) && (y2_ < 0 || y2_ > img.rows - 1) && (x2_ < 0 || x2_ > img.cols - 1))
	{
		tempPoints.push_back(Point2i(0, 0));
		tempPoints.push_back(Point2i(0, 0));
	}

	if (k <= 0);
	{
		d1 = GetDistanceP2P(Point2i(0, img.rows / 2), tempPoints[0]);
		d2 = GetDistanceP2P(Point2i(0, img.rows / 2), tempPoints[1]);
		if (d1 < d2)
		{
			edgePoints[0] = tempPoints[0];
			edgePoints[1] = tempPoints[1];
		}
		else
		{
			edgePoints[0] = tempPoints[1];
			edgePoints[1] = tempPoints[0];
		}

	}

	if (k > 0)
	{
		d1 = GetDistanceP2P(Point2i(0, 0), tempPoints[0]);
		d2 = GetDistanceP2P(Point2i(0, 0), tempPoints[1]);
		if (d1 < d2)
		{
			edgePoints[0] = tempPoints[0];
			edgePoints[1] = tempPoints[1];
		}
		else
		{
			edgePoints[0] = tempPoints[1];
			edgePoints[1] = tempPoints[0];
		}
	}
}

/***直线分类***/
void LineClassify(vector<Vec4i> originalLines, vector<Vec6i> &classifiedOrLines, vector<Vec4i> extendedLines, vector<Vec6i> &classifiedExLines, int gap)
{
	int length;
	vector<Vec4i> exLines_ = extendedLines;
	for (int i = 0; i < extendedLines.size(); i++)
	{
		Vec4i line1 = extendedLines[i];
		for (int j = 0; j < exLines_.size(); j++)
		{
			Vec4i line2 = exLines_[j];
			Vec4i line3 = originalLines[j];
			if (((abs(line1[0] - line2[0]) <= gap) && (abs(line1[1] - line2[1]) <= gap) && (abs(line1[2] - line2[2]) <= gap) && (abs(line1[3] - line2[3]) <= gap)))
			{
				classifiedExLines.push_back(Vec6i(line2[0], line2[1], line2[2], line2[3], i, 1));
				classifiedOrLines.push_back(Vec6i(line3[0], line3[1], line3[2], line3[3], i, 1));
				exLines_[j] = Vec4i(-999, -999, -999, -999);
			}
		}
	}
}

/***寻找直线点***/
void FindLinePoint(Mat img, Vec2d line, Point2i startPoint, Point2i &linePoint, int step)
{
	Point2i pt1, pt2, pt3;
	Point2i startPt = Point2i(-999, -999);
	Point2i endPt = Point2i(-999, -999);;
	int num = 0;
	int code1 = -1;
	int code2 = -1;
	double k = line[0];
	double b = line[1];

	while (abs(num) < 20)
	{
		if (k > 1 || k < -1)
		{
			pt1.y = startPoint.y + num;
			pt1.x = (int)((pt1.y - b) / k);
			if (pt1.x >= 0 && pt1.x < img.cols && pt1.y >= 0 && pt1.y < img.rows)
			{
				code1 = img.at<uchar>(pt1);
			}
			else
			{
				code1 = -1;
			}

			pt2.y = startPoint.y + num + step;
			pt2.x = (int)((pt2.y - b) / k);
			if (pt2.x >= 0 && pt2.x < img.cols && pt2.y >= 0 && pt2.y < img.rows)
			{
				code2 = img.at<uchar>(pt2);
			}
			else
			{
				code2 = -1;
			}
		}
		else
		{
			pt1.x = startPoint.x + num;
			pt1.y = (int)(k * pt1.x + b);
			if (pt1.x >= 0 && pt1.x < img.cols && pt1.y >= 0 && pt1.y < img.rows)
			{
				code1 = img.at<uchar>(pt1);
			}
			else
			{
				code1 = -1;
			}

			pt2.x = startPoint.x + num + step;
			pt2.y = (int)(k * pt2.x + b);
			if (pt2.x >= 0 && pt2.x < img.cols && pt2.y >= 0 && pt2.y < img.rows)
			{
				code2 = img.at<uchar>(pt2);
			}
			else
			{
				code2 = -1;
			}
		}
		if (code1 <= 0 && code2 == 255)
		{
			startPt = pt2;
		}
		else if (startPt.x>=0 && code1 == 255 && code2 <= 0)
		{
			endPt = pt1;
			break;
		}
		num += step;
	}

	pt3.x = (startPt.x + endPt.x) / 2;
	pt3.y = (startPt.y + endPt.y) / 2;

	if (pt3.x >= 0 && pt3.x < img.cols && pt3.y >= 0 && pt3.y < img.rows)
	{
		linePoint = pt3;
	}
	else
	{
		linePoint = Point2i(-999, -999);
	}

}

/***直线寻找策略***/
void SolveFLP(Mat cannyImg, Vec3d linePara, Vec6i startLine, Vec6i &optLine)
{
	double k, b1, b2, d1, d2;
	int startCode, endCode, x1, x2, y1, y2;

	vector<Point2i> startLinePts;
	vector<Point2i> endLinePts;
	Point2i pt1, pt2, pt3, startPt, endPt;
	Point2i noPt = Point2i(-999, -999);

	startPt = Point2i(startLine[0], startLine[1]);
	endPt = Point2i(startLine[2], startLine[3]);

	startCode = cannyImg.at<uchar>(startPt);
	endCode = cannyImg.at<uchar>(endPt);

	k = linePara[0];
	b1 = linePara[1];
	b2 = linePara[2];

	if (startCode == 255)
	{
		startLinePts.push_back(startPt);
		FindLinePoint(cannyImg, Vec2d(k, b1), startPt, pt1, 1);
		FindLinePoint(cannyImg, Vec2d(k, b1), startPt, pt2, -1);
		if (pt1.x >= 0 && pt2.x >= 0)
		{
			d1 = GetDistanceP2P(pt1, startPt);
			d2 = GetDistanceP2P(pt2, startPt);
			if (d1 < d2)
			{
				startLinePts.push_back(pt1);
			}
			else
			{
				startLinePts.push_back(pt2);
			}
		}
		else if (pt1.x >= 0)
		{
			startLinePts.push_back(pt1);
		}
		else if (pt2.x >= 0)
		{
			startLinePts.push_back(pt2);
		}
		else
		{
			startLinePts.push_back(startPt);
		}
	}
	else
	{
		FindLinePoint(cannyImg, Vec2d(k, b1), startPt, pt1, 1);
		FindLinePoint(cannyImg, Vec2d(k, b1), startPt, pt2, -1);

		if (pt1.x >= 0 && pt2.x >= 0)
		{
			startLinePts.push_back(pt1);
			startLinePts.push_back(pt2);
		}
		else if (pt1.x >= 0)
		{
			startLinePts.push_back(pt1);
			FindLinePoint(cannyImg, Vec2d(k, b1), pt1, pt3, 1);
			if (pt3.x >= 0)
			{
				startLinePts.push_back(pt3);
			}
			else
			{
				startLinePts.push_back(noPt);
			}
		}
		else if (pt2.x >= 0)
		{
			startLinePts.push_back(pt2);
			FindLinePoint(cannyImg, Vec2d(k, b1), pt2, pt3, -1);
			if (pt3.x >= 0)
			{
				startLinePts.push_back(pt3);
			}
			else
			{
				startLinePts.push_back(noPt);
			}
		}
		else
		{
			startLinePts.push_back(noPt);
			startLinePts.push_back(noPt);
		}
	}

	if (endCode == 255)
	{
		endLinePts.push_back(endPt);
		FindLinePoint(cannyImg, Vec2d(k, b2), endPt, pt1, 1);
		FindLinePoint(cannyImg, Vec2d(k, b2), endPt, pt2, -1);
		if (pt1.x >= 0 && pt2.x >= 0)
		{
			d1 = GetDistanceP2P(pt1, endPt);
			d2 = GetDistanceP2P(pt2, endPt);
			if (d1 < d2)
			{
				endLinePts.push_back(pt1);
			}
			else
			{
				endLinePts.push_back(pt2);
			}
		}
		else if (pt1.x >= 0)
		{
			endLinePts.push_back(pt1);
		}
		else if (pt2.x >= 0)
		{
			endLinePts.push_back(pt2);
		}
		else
		{
			endLinePts.push_back(endPt);
		}
	}
	else
	{
		FindLinePoint(cannyImg, Vec2d(k, b2), endPt, pt1, 1);
		FindLinePoint(cannyImg, Vec2d(k, b2), endPt, pt2, -1);

		if (pt1.x >= 0 && pt2.x >= 0)
		{
			endLinePts.push_back(pt1);
			endLinePts.push_back(pt2);
		}
		else if (pt1.x >= 0)
		{
			endLinePts.push_back(pt1);
			FindLinePoint(cannyImg, Vec2d(k, b2), pt1, pt3, 1);
			if (pt3.x >= 0)
			{
				endLinePts.push_back(pt3);
			}
			else
			{
				endLinePts.push_back(noPt);
			}
		}
		else if (pt2.x >= 0)
		{
			endLinePts.push_back(pt2);
			FindLinePoint(cannyImg, Vec2d(k, b2), pt2, pt3, -1);
			if (pt3.x >= 0)
			{
				endLinePts.push_back(pt3);
			}
			else
			{
				endLinePts.push_back(noPt);
			}
		}
		else
		{
			endLinePts.push_back(noPt);
			endLinePts.push_back(noPt);
		}
	}

	if (startLinePts[0].x >= 0 && startLinePts[1].x >= 0 && endLinePts[0].x >= 0 && endLinePts[1].x >= 0)
	{
		x1 = (int)((startLinePts[0].x + startLinePts[1].x) / 2);
		y1 = (int)((startLinePts[0].y + startLinePts[1].y) / 2);
		x2 = (int)((endLinePts[0].x + endLinePts[1].x) / 2);
		y2 = (int)((endLinePts[0].y + endLinePts[1].y) / 2);
		optLine = Vec6i(x1, y1, x2, y2, startLine[4], startLine[5]);
	}
	else
	{
		optLine = Vec6i(-999, -999, -999, -999, startLine[4], startLine[5]);
	}
}

/***直线优化***/
void LinesOptimization(Mat cannyImg, vector<Vec6i> lines, vector<Vec6i> exLines, vector<Vec6i> &optLinePoints, vector<Vec4i> &optimizedLines)
{

	int startX, startY, endX, endY, firstX, firstY, lastX, lastY, x1, x2, y1, y2, \
		sX, sY, eX, eY, fX, fY, lX, lY, startCode, endCode, firstCode, lastCode;
	double k, b, k_v, b_v1, b_v2, b_v3, b_v4, d1, d2;
	Point2i pt1, pt2, pt3, startPt, endPt, firstPt, lastPt;
	Point2i noPt = Point2i(-999, -999);
	Vec6i line, exLine, optLine1, optLine2;
	vector<Point2i> startLinePts;
	vector<Point2i> endLinePts;
	vector<Point2i> firstLinePts;
	vector<Point2i> lastLinePts;
	vector<Vec6i> optLines;

	for (int i = 0; i < lines.size(); i++)
	{
		line = lines[i];
		exLine = exLines[i];
		startX = line[0];
		startY = line[1];
		endX = line[2];
		endY = line[3];
		firstX = exLine[0];
		firstY = exLine[1];
		lastX = exLine[2];
		lastY = exLine[3];


		k = (double)(endY - startY) / (double)(endX - startX + 0.00001);    //直线斜率
		b = startY - k * startX;									    	//直线截距

		sX = (int)((2 * startX + endX) / 3);
		sY = (int)((2 * startY + endY) / 3);
		eX = (int)((2 * endX + startX) / 3);
		eY = (int)((2 * endY + startY) / 3);

		if (k > 1 || k < -1)
		{
			if (firstY < lastY)
			{
				fY = firstY + 10;
				lY = lastY - 10;
				fX = (int)((fY - b) / k);
				lX = (int)((lY - b) / k);
			}
			else
			{
				fY = firstY - 10;
				lY = lastY + 10;
				fX = (int)((fY - b) / k);
				lX = (int)((lY - b) / k);
			}
		}
		else
		{
			if (firstX < lastX)
			{
				fX = firstX + 10;
				lX = lastX - 10;
				fY = (int)(k * fX + b);
				lY = (int)(k * lX + b);
			}
			else
			{
				fX = firstX - 10;
				lX = lastX + 10;
				fY = (int)(k * fX + b);
				lY = (int)(k * lX + b);
			}
		}

		k_v = -1 / k;												    	//垂直直线斜率
		b_v1 = sY - k_v * sX;									        	//垂直直线截距
		b_v2 = eY - k_v * eX;										        //垂直直线截距
		b_v3 = fY - k_v * fX;									            //垂直直线截距
		b_v4 = lY - k_v * lX;										        //垂直直线截距

		startPt = Point2i(sX, sY);
		endPt = Point2i(eX, eY);
		firstPt = Point2i(startX, startY);
		lastPt = Point2i(endX, endY);

		SolveFLP(cannyImg, Vec3d(k_v, b_v1, b_v2), Vec6i(sX, sY, eX, eY, line[4], line[5]), optLine1);
		SolveFLP(cannyImg, Vec3d(k_v, b_v3, b_v4), Vec6i(fX, fY, lX, lY, line[4], line[5]), optLine2);

		if (optLine1[0] >= 0)
		{
			optLines.push_back(optLine1);
		}

		if (optLine2[0] >= 0)
		{
			optLines.push_back(optLine2);
		}

	}

	optLinePoints = optLines;

	int optX1, optX2, optY1, optY2;
	double optX, optY, optK;
	Vec6i line1;
	Vec6i line2;
	Vec4f lineTemp;
	vector<Point2i> ptsTemp;

	sort(optLines.begin(), optLines.end(), CompByPID);

	for (int i = 0; i < optLines.size(); i++)
	{
		if (i == optLines.size() - 1)
		{
			line1 = optLines[i];
			line2 = Vec6i(line1[0], line1[1], line1[2], line1[3], -1, 1);
		}
		else
		{
			line1 = optLines[i];
			line2 = optLines[i + 1];
		}

		x1 = line1[0];
		y1 = line1[1];
		x2 = line1[2];
		y2 = line1[3];

		ptsTemp.push_back(Point2i(x1, y1));
		ptsTemp.push_back(Point2i(x2, y2));

		if (line1[4] != line2[4])
		{
			fitLine(ptsTemp, lineTemp, DIST_L2, 0, 0.01, 0.01);
			optK = lineTemp[1] / lineTemp[0];
			optX = lineTemp[2];
			optY = lineTemp[3];
			optX1 = x1;
			optY1 = optK * (optX1 - optX) + optY;
			optX2 = x2;
			optY2 = optK * (optX2 - optX) + optY;
			optimizedLines.push_back(Vec4i(optX1, optY1, optX2, optY2));
			ptsTemp.clear();
		}
	}


}

/***角点获取***/
void GetCornerPoints(Mat img, vector<Vec4i> &lines, vector<Point2i> &cornerPoints, vector<Vec4d> &intersectLines)
{
	for (int i = 0; i < lines.size(); i++)
	{
		Vec4i line1 = lines[i];
		double k1 = (double)(line1[3] - line1[1]) / (double)(line1[2] - line1[0] + 0.00001); //直线斜率
		double b1 = line1[1] - k1 * line1[0]; //直线截距
		for (int j = i + 1; j < lines.size(); j++)
		{
			Vec4i line2 = lines[j];
			double k2 = (double)(line2[3] - line2[1]) / (double)(line2[2] - line2[0] + 0.00001); //直线斜率
			double b2 = line2[1] - k2 * line2[0]; //直线截距

			//交点坐标
			double x = -(b2 - b1) / (k2 - k1);
			double y = k1 * x + b1;
			if (x >= 0 && x < img.cols && y >= 0 && y < img.rows)
			{
				cornerPoints.push_back(Point2i((int)x, (int)y));
				intersectLines.push_back(Vec4d(k1, b1, k2, b2));
			}
		}
		

	}
}

/***沿直线搜寻编码点***/
void FindCodingPointByLine(Mat img, Vec2d line, Point2i startPoint, Point2i &codingPoint, int &colorCode, int step)
{
	Point2i pt1, pt2, pt3;
	Point2i startPt = Point2i(-999, -999);
	Point2i endPt = Point2i(-999, -999);;
	int num = 0;
	int code1 = -1;
	int code2 = -1;
	double k = line[0];
	double b = line[1];


	while (abs(num) < 200)
	{
		if (k > 1 || k < -1)
		{
			pt1.y = startPoint.y + num;
			pt1.x = (int)((pt1.y - b) / k);
			if (pt1.x >= 0 && pt1.x < img.cols && pt1.y >= 0 && pt1.y < img.rows)
			{
				code1 = GetColorCode(img, pt1);
			}
			else
			{
				code1 = -1;
			}

			pt2.y = startPoint.y + num + step;
			pt2.x = (int)((pt2.y - b) / k);
			if (pt2.x >= 0 && pt2.x < img.cols && pt2.y >= 0 && pt2.y < img.rows)
			{
				code2 = GetColorCode(img, pt2);
			}
			else
			{
				code2 = -1;
			}
		}
		else
		{
			pt1.x = startPoint.x + num;
			pt1.y = (int)(k * pt1.x + b);
			if (pt1.x >= 0 && pt1.x < img.cols && pt1.y >= 0 && pt1.y < img.rows)
			{
				code1 = GetColorCode(img, pt1);
			}
			else
			{
				code1 = -1;
			}

			pt2.x = startPoint.x + num + step;
			pt2.y = (int)(k * pt2.x + b);
			if (pt2.x >= 0 && pt2.x < img.cols && pt2.y >= 0 && pt2.y < img.rows)
			{
				code2 = GetColorCode(img, pt2);
			}
			else
			{
				code2 = -1;
			}
		}
		if (code1 < 0 && code2 > 0)
		{
			startPt = pt2;
		}
		else if (startPt.x >= 0 && code1 > 0 && code2 < 0)
		{
			endPt = pt1;
			break;
		}
		num += step;
	}

	pt3.x = (startPt.x + endPt.x) / 2;
	pt3.y = (startPt.y + endPt.y) / 2;

	if (pt3.x >= 0 && pt3.x < img.cols && pt3.y >= 0 && pt3.y < img.rows)
	{
		codingPoint = pt3;
		colorCode = GetColorCode(img, pt3);
	}
	else
	{
		codingPoint = Point2i(-999, -999);
		colorCode = -1;
	}
}

/***寻找初始编码点***/
void FindFirstCodingPoints(Mat hsvImg, vector<Point2i> cornerPoints, vector<Vec4d> &intersectLines, vector<Vec4d> &findingVectors, vector<Point2i> &codingPoints)
{
	Point2i pt;
	double k1, k2, b1, b2;
	int colorCode = -1;

	for (int i = 0; i < cornerPoints.size(); i++)
	{
		k1 = intersectLines[i][0];
		b1 = intersectLines[i][1];
		k2 = intersectLines[i][2];
		b2 = intersectLines[i][3];

		FindCodingPointByLine(hsvImg, Vec2d(k1,b1), cornerPoints[i], pt, colorCode, 1);
		if (colorCode > 0)
		{
			findingVectors.push_back(Vec4d(k1, b1, k2, b2));
			codingPoints.push_back(pt);
			continue;
		}

		FindCodingPointByLine(hsvImg, Vec2d(k2, b2), cornerPoints[i], pt, colorCode, 1);
		if (colorCode > 0)
		{
			findingVectors.push_back(Vec4d(k2, b2, k1, b1));
			codingPoints.push_back(pt);
			continue;
		}

		if (colorCode < 0)
		{
			findingVectors.push_back(Vec4d(0, 0, 0, 0));
			codingPoints.push_back(Point2i(-999, -999));
		}
	}
}

/***计算角平分线***/
void GetAngleBisector(Vec4d inputLines, Vec4d &outputLines)
{
	double a1 = inputLines[0];
	double b1 = -1;
	double c1 = inputLines[1];

	double a2 = inputLines[2];
	double b2 = -1;
	double c2 = inputLines[3];

	double a3 = a1 * sqrt(a2 * a2 + b2 * b2) - a2 * sqrt(a1 * a1 + b1 * b1);
	double b3 = b1 * sqrt(a2 * a2 + b2 * b2) - b2 * sqrt(a1 * a1 + b1 * b1);
	double c3 = c1 * sqrt(a2 * a2 + b2 * b2) - c2 * sqrt(a1 * a1 + b1 * b1);

	double a4 = a1 * sqrt(a2 * a2 + b2 * b2) + a2 * sqrt(a1 * a1 + b1 * b1);
	double b4 = b1 * sqrt(a2 * a2 + b2 * b2) + b2 * sqrt(a1 * a1 + b1 * b1);
	double c4 = c1 * sqrt(a2 * a2 + b2 * b2) + c2 * sqrt(a1 * a1 + b1 * b1);

	double k_3 = -a3 / b3;
	double b_3 = -c3 / b3;
	double k_4 = -a4 / b4;
	double b_4 = -c4 / b4;

	outputLines[0] = k_3;
	outputLines[1] = b_3;
	outputLines[2] = k_4;
	outputLines[3] = b_4;


}

/***生成角点最终编码***/
void GenerateFinalCode(vector<Point2i> cornerPoints, vector<Vec4i> codingpoints, vector<Point2i> firstCodingPoints, vector<Vec3i> &codingCornerPoints)
{
	vector<int> pointsFlag;
	vector<vector<Vec4i>> PointsAngle;
	vector<Vec4i> PtsAngle;
	Vec3i codingCornerPoint;
	Vec4i vec1, vec2;
	Point2i pt1, pt2, c;
	string code = "";
	int colorCode;
	int pFlag;
	int pAngle;

	//判断点群中是否存在未识别编码
	for (int i = 0; i < codingpoints.size() / 5; i++)
	{
		for (int j = 0; j < 5; j++)
		{
			colorCode = codingpoints[i * 5+ j][2];
			if (colorCode < 0)
			{
				pFlag = -1;
				break;
			}
			else
			{
				pFlag = 1;
			}
		}
		pointsFlag.push_back(pFlag);
	}

	//计算编码点与初始编码点夹角
	for (int i = 0; i < codingpoints.size() / 5; i++)
	{
		if (pointsFlag[i]>0)
		{
			for (int j = 0; j < 5; j++)
			{
				c = cornerPoints[i];
				pt1 = firstCodingPoints[i];
				pt2 = Point2i(codingpoints[i * 5 + j][0], codingpoints[i * 5 + j][1]);
				pAngle = (int)GetAngleOfTwoVector(pt1, pt2, c);
				colorCode = codingpoints[i * 5 + j][2];
				PtsAngle.push_back(Vec4i(pt2.x, pt2.y, colorCode, pAngle));
			}
			PointsAngle.push_back(PtsAngle);
			PtsAngle.clear();
		}
		else
		{
			for (int j = 0; j < 5; j++)
			{
				PtsAngle.push_back(Vec4i(-1, -1, -1, -1));
			}
			PointsAngle.push_back(PtsAngle);
			PtsAngle.clear();

		}
	}

	//按角度排序
	for (int i = 0; i < PointsAngle.size(); i++)
	{
		sort(PointsAngle[i].begin(), PointsAngle[i].end(), CompByAngle);
	}

	//生成最终编码
	for (int i = 0; i < PointsAngle.size(); i++)
	{
		for (int j = 0; j < PointsAngle[0].size(); j++)
		{
			if (PointsAngle[i][j][2] > 0)
			{
				code += to_string(PointsAngle[i][j][2]);
			}
			else
			{
				code = "0";
			}

		}

		codingCornerPoint = Vec3i(cornerPoints[i].x, cornerPoints[i].y, stoi(code));
		codingCornerPoints.push_back(codingCornerPoint);
		code = "";
	}

}

/***编码转换坐标***/
void GetCodeXY(vector<Vec3i> ccFilterPts, vector<Vec4f> codePts, vector<Point3f> &wPts)
{
	int code;
	Point3f pt;
	map<string, Point3f> codeMap;
	for (int i = 0; i < codePts.size(); i++)
	{
		string s = to_string((int)codePts[i][3]);
		codeMap[s] = Point3f(codePts[i][0], codePts[i][1], codePts[i][2]);
	}

	for (int i = 0; i < ccFilterPts.size(); i++)
	{
		code = ccFilterPts[i][2];

		string str;
		if (code == 0)
		{
			pt = Point3f(-999, -999, -999);
		}
		else
		{
			str = to_string(code);
			map<string, Point3f>::iterator iter1, iter2;
			iter1 = codeMap.find(str);
			if (iter1 != codeMap.end())
			{
				pt = codeMap[str];
			}
		}
		wPts.push_back(pt);
	}
}

/***后方交会P3P***/
void RunResection(vector<Vec3i> codingCornerPts, vector<Point3f> wPts, Mat cameraIntrinsicMatrix, Mat distCoeffs, int downSampleTimes)
{
	//初始化降采样系数阵
	int n = downSampleTimes;
	double downSampleCoeffs[9] = {
		     1/(2.*n), 0       , 0      ,
			 0       , 1/(2.*n), 0      ,
			 0       , 0       , 1/(2.*n)};

	Mat dsc = Mat(3, 3, CV_64FC1, downSampleCoeffs);
	Mat camIntrinsicMatrix = dsc * cameraIntrinsicMatrix;

	//特征点像素坐标
	vector<cv::Point2f> Points2D;
	for (int i = 0; i < codingCornerPts.size(); i++)
	{
		Points2D.push_back(Point2i(codingCornerPts[i][0], codingCornerPts[i][1]));
	}

	//特征点世界坐标
	vector<cv::Point3f> Points3D;
	for (int i = 0; i < wPts.size(); i++)
	{
		Points3D.push_back(wPts[i]);     
	}

	//初始化输出矩阵
	Mat rvec;
	Mat tvec;

	//求解PNP
	solvePnP(Points3D, Points2D, camIntrinsicMatrix, distCoeffs, rvec, tvec, false, SOLVEPNP_ITERATIVE);                //Number of input points must be >= 4 and object points can be in any configuration
	//solvePnP(Points3D, Points2D, camera_matrix, distortion_coefficients, rvec, tvec, false, SOLVEPNP_EPNP);			//Number of input points must be >= 4 and object points can be in any configuration
	//solvePnP(Points3D, Points2D, camera_matrix, distortion_coefficients, rvec, tvec, false, SOLVEPNP_AP3P);			//Need 4 input points to return a unique solution
	//solvePnP(Points3D, Points2D, camera_matrix, distortion_coefficients, rvec, tvec, false, SOLVEPNP_P3P);			//Need 4 input points to return a unique solution
	//solvePnP(Points3D, Points2D, camera_matrix, distortion_coefficients, rvec, tvec, false, SOLVEPNP_IPPE);			//Input points must be >= 4 and object points must be coplanar
 
	FileStorage result("result.yaml", FileStorage::WRITE); //输出结果文件

	Mat rotMat,P_oc;
	Rodrigues(rvec, rotMat);       //由于solvePnP返回的是旋转向量，故用罗德里格斯变换变成旋转矩阵
	cout << "旋转矩阵：" << endl << rotMat << endl;
	result << "RotateMatrix" << rotMat;

	P_oc = -rotMat.inv() * tvec;
	cout << "相机世界坐标：" << endl << P_oc << endl;
	result << "CameraPosition" << P_oc;




	//重投影
	vector<Point2f> reProjPoints2D;
	projectPoints(Points3D, rvec, tvec, camIntrinsicMatrix, distCoeffs, reProjPoints2D);

	float reProjErr = 0.0;
	for (int i = 0; i < Points2D.size(); i++)
	{
		reProjErr += sqrt(pow((reProjPoints2D[i].x - Points2D[i].x), 2) + pow((reProjPoints2D[i].y - Points2D[i].y), 2));
	}
	reProjErr /= Points2D.size();

	cout << "重投影误差：" << endl;
	cout << reProjErr << endl;
	result << "ReprojectError" << reProjErr;
	result.release();
}

/***影像预处理***/
void ImgPreProcess(Mat &img, Mat &cannyImg)
{
	//降采样
	pyrDown(img, img, Size(img.cols / 2, img.rows / 2));
	pyrDown(img, img, Size(img.cols / 2, img.rows / 2));
	imwrite("resizeImg.png", img);

	//预处理
	GaussianBlur(img, img, Size(3, 3), 0); //高斯模糊
	imwrite("gaussblurImg.png", img);


	//转灰度图
	Mat grayImg;
	cvtColor(img, grayImg, COLOR_BGR2GRAY);

	//Canny 边缘检测
	Canny(grayImg, cannyImg, 100, 200, 3);
	imwrite("cannyImg.png", cannyImg);

	//膨胀
	Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
	dilate(cannyImg, cannyImg, element);
	imwrite("dilateImg.png", cannyImg);


	//去除孤立区域
	double area;
	int areaThreshold = 400;
	vector<vector<Point2i> > contours;
	vector<Vec4i> hierarchy;
	findContours(cannyImg, contours, hierarchy, RETR_LIST, CHAIN_APPROX_NONE);  //寻找轮廓
	for (int i = 0; i < contours.size(); i++)
	{
		area = contourArea(contours[i]);
		if (area < areaThreshold)
		{
			drawContours(cannyImg, contours, i, Scalar(0), FILLED, 8, hierarchy);  //填充面积小于指定阈值的轮廓
		}
	}
	imwrite("eraseImg.png", cannyImg);

}

/***直线提取***/
void GetLines(Mat img, Mat cannyImg, vector<Vec4i> &extendOptLines)
{
	//Hough变换 直线提取
	vector<Vec4i> lines;
	HoughLinesP(cannyImg, lines, 1., CV_PI / 180, 300, 200, 30);
	//绘制
	Mat drawImg = img.clone();
	for (int i = 0; i < lines.size(); i++)
	{
		line(drawImg, Point2i(lines[i][0], lines[i][1]), Point2i(lines[i][2], lines[i][3]), Scalar(0, 0, 255), 1, LINE_AA);
	}
	imwrite("houghImg.png", drawImg);


	//原始直线延展
	vector<Point2i> edgePoints;
	vector<Vec4i> extendLines;
	for (int i = 0; i < lines.size(); i++) {
		LineExtend(img, lines[i], edgePoints);
		extendLines.push_back(Vec4i(edgePoints[0].x, edgePoints[0].y, edgePoints[1].x, edgePoints[1].y));
		edgePoints.clear();
	}
	//绘制
    drawImg = img.clone();
	for (int i = 0; i < extendLines.size(); i++)
	{
		line(drawImg, Point2i(extendLines[i][0], extendLines[i][1]), Point2i(extendLines[i][2], extendLines[i][3]), Scalar(0, 0, 255), 1, LINE_AA);
	}
	imwrite("extendLinesImg.png", drawImg);


	//直线分类
	vector<Vec6i> classifiedExLines;
	vector<Vec6i> classifiedOrLines;
	LineClassify(lines, classifiedOrLines, extendLines, classifiedExLines, 50);

	//直线优化
	vector<Vec6i> optLinePoints;
	vector<Vec4i> optimizedLines;
	LinesOptimization(cannyImg, classifiedOrLines, classifiedExLines, optLinePoints, optimizedLines);

	//绘制直线拟合点
	Mat linePts = img.clone();
	for (int i = 0; i < optLinePoints.size(); i++)
	{
		circle(linePts, Point2i(optLinePoints[i][0], optLinePoints[i][1]), 3, Scalar(0, 255, 255), -1);
		circle(linePts, Point2i(optLinePoints[i][2], optLinePoints[i][3]), 3, Scalar(0, 255, 255), -1);
	}
	imwrite("optLinePts.png", linePts);

	//绘制拟合后直线
	drawImg = img.clone();
	for (int i = 0; i < optimizedLines.size(); i++)
	{
		line(drawImg, Point2i(optimizedLines[i][0], optimizedLines[i][1]), Point2i(optimizedLines[i][2], optimizedLines[i][3]), Scalar(0, 0, 255), 2, LINE_AA);
	}
	imwrite("optLinesImg.png", drawImg);

	//优化后直线延展
	for (int i = 0; i < optimizedLines.size(); i++) {
		LineExtend(img, optimizedLines[i], edgePoints);
		extendOptLines.push_back(Vec4i(edgePoints[0].x, edgePoints[0].y, edgePoints[1].x, edgePoints[1].y));
		edgePoints.clear();
	}

	drawImg = img.clone();
	for (int i = 0; i < extendOptLines.size(); i++)
	{
		line(drawImg, Point2i(extendOptLines[i][0], extendOptLines[i][1]), Point2i(extendOptLines[i][2], extendOptLines[i][3]), Scalar(0, 0, 255), 2, LINE_AA);
	}
	imwrite("extendOptLinesImg.png", drawImg);
}

/***角点提取及编码识别***/
void GetCornerCodingPts(Mat img, vector<Vec4i> extendOptLines, vector<Point2i> &cornerPoints, vector<Vec4i> &codingPoints, vector<Vec3i> &ccFilterPts)
{
	//计算角点
	vector<Vec4d> intersectLines;
	GetCornerPoints(img, extendOptLines, cornerPoints, intersectLines);
	//绘制角点
	Mat drawImg = img.clone();
	for (int i = 0; i < extendOptLines.size(); i++)
	{
		line(drawImg, Point2i(extendOptLines[i][0], extendOptLines[i][1]), Point2i(extendOptLines[i][2], extendOptLines[i][3]), Scalar(0, 0, 255), 2, LINE_AA);
	}
	for (int i = 0; i < cornerPoints.size(); i++)
	{
		circle(drawImg, cornerPoints[i], 3, Scalar(0, 255, 0), -1);
	}
	imwrite("cornerPtsImg.png", drawImg);

	//转换颜色空间至HSV
	Mat hsvImg;
	cvtColor(img, hsvImg, COLOR_BGR2HSV);
	imwrite("hsvImg.png", hsvImg);


	//寻找初始编码点
	vector<Vec4d> findingVectors;
	vector<Point2i> firstCodingPoints;
	FindFirstCodingPoints(hsvImg, cornerPoints, intersectLines, findingVectors, firstCodingPoints);
	//绘制
	for (int i = 0; i < firstCodingPoints.size(); i++)
	{
		circle(drawImg, firstCodingPoints[i], 3, Scalar(0, 255, 255), -1);
	}
	imwrite("firstCodingPtsImg.png", drawImg);

	//寻找所有编码点(沿三条线)
	vector<Vec4d> angleBisectorLines;

	for (int i = 0; i < cornerPoints.size(); i++)
	{
		Vec4d abLine;
		Point2i pt;
		int ptCode;

		GetAngleBisector(findingVectors[i], abLine);  //计算角平分线
		angleBisectorLines.push_back(abLine);

		FindCodingPointByLine(hsvImg, Vec2d(findingVectors[i][0], findingVectors[i][1]), cornerPoints[i], pt, ptCode, 1);
		codingPoints.push_back(Vec4i(pt.x, pt.y, ptCode, i));

		FindCodingPointByLine(hsvImg, Vec2d(findingVectors[i][0], findingVectors[i][1]), cornerPoints[i], pt, ptCode, -1);
		codingPoints.push_back(Vec4i(pt.x, pt.y, ptCode, i));

		FindCodingPointByLine(hsvImg, Vec2d(abLine[0], abLine[1]), cornerPoints[i], pt, ptCode, 1);
		codingPoints.push_back(Vec4i(pt.x, pt.y, ptCode, i));

		FindCodingPointByLine(hsvImg, Vec2d(abLine[0], abLine[1]), cornerPoints[i], pt, ptCode, -1);
		codingPoints.push_back(Vec4i(pt.x, pt.y, ptCode, i));

		FindCodingPointByLine(hsvImg, Vec2d(abLine[2], abLine[3]), cornerPoints[i], pt, ptCode, 1);
		codingPoints.push_back(Vec4i(pt.x, pt.y, ptCode, i));

		FindCodingPointByLine(hsvImg, Vec2d(abLine[2], abLine[3]), cornerPoints[i], pt, ptCode, -1);
		codingPoints.push_back(Vec4i(pt.x, pt.y, ptCode, i));

	}

	//编码点滤除
	vector<Vec4i> cFilterPts;
	for (int i = 0; i < codingPoints.size(); i++)
	{
		if (codingPoints[i][2] > 0)
		{
			cFilterPts.push_back(codingPoints[i]);
		}
	}
	//绘制
	for (int i = 0; i < cFilterPts.size(); i++)
	{
		circle(drawImg, Point2i(cFilterPts[i][0], cFilterPts[i][1]), 3, Scalar(0, 255, 255), -1);
		putText(drawImg, to_string(cFilterPts[i][2]), Point2i(cFilterPts[i][0], cFilterPts[i][1]), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 3, 8);
	}
	imwrite("CodingPtsImg.png", drawImg);

	//生成最终编码
	vector<Vec3i> codingCornerPoints;
	GenerateFinalCode(cornerPoints, cFilterPts, firstCodingPoints, codingCornerPoints);

	//编码点二次滤除
	for (int i = 0; i < codingCornerPoints.size(); i++)
	{
		if (codingCornerPoints[i][2] > 0)
		{
			ccFilterPts.push_back(codingCornerPoints[i]);
		}
	}
}

int main()
{
	/**********读取配置文件**********/
	FileStorage config("config.yaml", FileStorage::READ);

	string imgPath;
	config["imgPath"] >> imgPath;  //图像路径

	int downSampleTimes;
	config["downSampleTimes"] >> downSampleTimes;  //降采样次数

	Mat cameraIntrinsicMatrix, distCoeffs;
	config["cameraIntrinsicMatrix"] >> cameraIntrinsicMatrix;  //内参矩阵
	config["distCoffes"] >> distCoeffs;  //畸变系数

	FileNode codepts = config["codePoints"];  //编码点坐标
	FileNodeIterator it = codepts.begin(), it_end = codepts.end();
	vector<Vec4f> codePts;

	for (int i = 0; it != it_end; it++, i++)
	{
		codePts.push_back(Vec4f((float)(*it)["x"], (float)(*it)["y"], (float)(*it)["z"], (float)(*it)["code"]));
	}

	config.release();
	/**********开始影像预处理**********/
	Mat img, cannyImg;
	Mat imgSrc = imread(imgPath);
	imgSrc.copyTo(img);

	ImgPreProcess(img, cannyImg);

	/**********开始直线及编码点提取**********/
	vector<Point2i> cornerPoints;
	vector<Vec4i> codingPoints;
	vector<Vec4i> extendOptLines;
	vector<Vec3i> ccFilterPts;

	//直线提取
	GetLines(img, cannyImg, extendOptLines);

	//角点提取及编码识别
	GetCornerCodingPts(img, extendOptLines, cornerPoints, codingPoints, ccFilterPts);

	//获取编码点物方坐标
	vector<Point3f> wPts;
	GetCodeXY(ccFilterPts, codePts, wPts);

	//进行后方交会
	RunResection(ccFilterPts, wPts, cameraIntrinsicMatrix, distCoeffs, downSampleTimes);

	/**********图像绘制与结果输出**********/
	//绘制地板缝线
	for (int i = 0; i < extendOptLines.size(); i++)
	{
		line(img, Point2i(extendOptLines[i][0], extendOptLines[i][1]), Point2i(extendOptLines[i][2], extendOptLines[i][3]), Scalar(0, 0, 255), 2, LINE_AA);
	}

	////绘制编码点
	//for (int i = 0; i < codingPoints.size(); i++)
	//{
	//	circle(img, Point2i(codingPoints[i][0], codingPoints[i][1]), 3, Scalar(0, 255, 255), -1);
	//	putText(img, to_string(codingPoints[i][2]), Point2i(codingPoints[i][0], codingPoints[i][1]), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 3, 8);
	//}

	//绘制角点
	for (int i = 0; i < ccFilterPts.size(); i++)
	{
		circle(img, Point2i(ccFilterPts[i][0], ccFilterPts[i][1]), 3, Scalar(0, 255, 0), -1);
	}
    
	//绘制角点最终编码
	for (int i = 0; i < ccFilterPts.size(); i++)
	{
		stringstream stream1, stream2, stream3;
		stream1 << fixed << setprecision(2) << wPts[i].x;
		stream2 << fixed << setprecision(2) << wPts[i].y;
		stream3 << fixed << setprecision(2) << wPts[i].z;
		string s1 = stream1.str();
		string s2 = stream2.str();
		string s3 = stream3.str();
		string text = "(" + s1 + "," + s2 + "," + s3 + ")";
		putText(img, text, Point2i(ccFilterPts[i][0], ccFilterPts[i][1]), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 2, 8);
	}

	//图像显示
	namedWindow("IMG", WINDOW_NORMAL);
	imshow("IMG", img);
	waitKey(0);

	//写出
	imwrite("result.jpg", img);

}

