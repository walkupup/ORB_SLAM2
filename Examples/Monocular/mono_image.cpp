/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/opencv.hpp>

#include<System.h>
#ifdef WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#define COMPILEDWITHC11
using namespace std;
using namespace cv;

void DrawTrajectory(cv::Mat &mShowImg, std::vector<cv::Point2f> mPts)
{
	int width = mShowImg.cols;
	int height = mShowImg.rows;
	static int basex = width / 2;
	static int basey = height * 4 / 5;
	static float showScale = 10;
	int rangePix = (width < height ? width : height);//std::min(width, height);
	int margin = rangePix / 4;
	int x, y;

	if (mPts.size() < 1)
		return;
	//cout << mPts.size() << endl;
	// 得到最小最大范围
	float minx = mPts[0].x, maxx = mPts[0].x, miny = mPts[0].y, maxy = mPts[0].y;
	for (int i = 1; i < mPts.size(); i++)
	{
		if (mPts[i].x < minx)
			minx = mPts[i].x;
		if (mPts[i].x > maxx)
			maxx = mPts[i].x;
		if (mPts[i].y < miny)
			miny = mPts[i].y;
		if (mPts[i].y > maxy)
			maxy = mPts[i].y;
	}

	// 确定显示比例
	float range = maxx - minx;
	if (maxy - miny > range)
		range = maxy - miny;
	if (range < 0.1)
		range = 0.1;
	if (range * showScale > rangePix - margin * 2)
		showScale = (rangePix - margin * 2) / range;
	if (range * showScale < margin * 2)
		showScale = margin * 2 / range;

	// 确定基准点，公式： bx + minx * scale = margin, by - miny * scale = height - margin
	basex = margin - minx * showScale;
	basey = (height - margin) + miny * showScale;

	// 画点
	mShowImg.setTo(255);
	for (int i = 0; i < mPts.size() - 1; i++)
	{
		circle(mShowImg, cv::Point(mPts[i].x * showScale + basex, -mPts[i].y * showScale + basey), 1, cv::Scalar(120, 120, 120), 1);
		line(mShowImg, cv::Point(mPts[i].x * showScale + basex, -mPts[i].y * showScale + basey),
			cv::Point(mPts[i + 1].x * showScale + basex, -mPts[i + 1].y * showScale + basey), cv::Scalar(120, 120, 120), 1);
	}
}



int main(int argc, char **argv)
{
	char *param = "jy-fish.yaml";
	//char *param = "redmi.yaml";
	int start = 100;// 1266;
	
	// 
	std::string path = "../../data/image0224";
	//std::string path = "../../data/image-backforth";
	//std::string path = "G:/data/slam/image-backforth";

	char *voc = "../../Vocabulary/ORBvoc.bin";
	//char *voc = "../Vocabulary/ORBvoc.bin";

	cv::Mat showImg(600, 800, CV_8UC1);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(voc, param, ORB_SLAM2::System::MONOCULAR, false);

    cout << "Start processing  ..." << endl;

    // Main loop
    cv::Mat im, grayim;
	cv::Mat halfim;
	char name[100];
	std::vector<cv::Point2f> Pts;
    for(int ni= start; ni<10000; ni+=3)
    {
        // Read image from file
        //im = cv::imread(string(argv[3])+"/"+vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
		//printf("width %d , height %d\n", im.cols, im.rows);
		sprintf(name, "%s/image%d.jpg", path.c_str(), ni);
		grayim = imread(name, CV_LOAD_IMAGE_GRAYSCALE);
		if (grayim.empty())
			continue;
		double tframe = ni;// vTimestamps[ni];

		double tt = (double)getTickCount();

        // Pass the image to the SLAM system
		//cvtColor(im, grayim, CV_BGR2GRAY);
		//resize(grayim, halfim, Size(im.cols / 2, im.rows / 2));
		Point3f pt;
		float quality;
        SLAM.TrackMonocular(grayim,tframe, pt, quality);
		if (quality > 0)
			Pts.push_back(Point2f(pt.x, pt.y));

		printf("q%.2f\t%d\t%.1f\n", quality, ni, ((double)getTickCount() - tt) / getTickFrequency() * 1000);
		showImg.setTo(255);
		//SLAM.DrawKeyFrameTrajectoryTUM(showImg, 2);
		////SLAM.DrawKeyFrameAll(showImg, 2, 300);
		DrawTrajectory(showImg, Pts);
		imshow("traj", showImg);
		imshow("image", grayim);
		waitKey(1);


        //double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

	}

    // Stop all threads
    SLAM.Shutdown();
	waitKey(0);

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}


