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

int main(int argc, char **argv)
{
	//VideoCapture cap("jy-vga.avi");
	//char *param = "jy-fish-vga.yaml";

	VideoCapture cap("jy1.avi");
	char *param = "jy-fish.yaml";


	char *voc = "../../../Vocabulary/ORBvoc.bin";
	//char *voc = "../../../Vocabulary/ORBvoc.bin";
	//char *param = "redmi.yaml";
	int width = 1280;
	int height = 720;
	cv::Mat showImg(600, 800, CV_8UC1);
	namedWindow("traj", CV_WINDOW_FULLSCREEN);

	if (!cap.isOpened())
	{
		printf("video not found\n");
		return 0;
	}
	//cap.set(CV_CAP_PROP_FRAME_WIDTH, width);
	//cap.set(CV_CAP_PROP_FRAME_HEIGHT, height);
	printf("cam width = %d, height = %d\n", (int)cap.get(CV_CAP_PROP_FRAME_WIDTH), (int)cap.get(CV_CAP_PROP_FRAME_HEIGHT));
	//VideoWriter record("f.avi", -1, 15, Size(width, height));

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(voc, param, ORB_SLAM2::System::MONOCULAR, false);
	//SLAM.ActivateLocalizationMode();

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;

    // Main loop
    cv::Mat im, grayim;
	cv::Mat halfim;
    for(int ni=0; ni<10000; ni++)
    {
        // Read image from file
        //im = cv::imread(string(argv[3])+"/"+vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
		//cap >> im;
		//cap >> im;
		cap >> im;
		//record << im;
		//printf("width %d , height %d\n", im.cols, im.rows);
		if (im.empty())
			break;
		double tframe = ni;// vTimestamps[ni];

		double tt = (double)getTickCount();

        // Pass the image to the SLAM system
		cvtColor(im, grayim, CV_BGR2GRAY);
		//resize(grayim, halfim, Size(im.cols / 2, im.rows / 2));
        SLAM.TrackMonocular(grayim,tframe);

		printf("%d\t%.1f\n", ni, ((double)getTickCount() - tt) / getTickFrequency() * 1000);
		//showImg.setTo(255);
		SLAM.DrawKeyFrameTrajectoryTUM(showImg, 2);
		//SLAM.DrawKeyFrameAll(showImg, 2, 300);
		imshow("traj", showImg);
		imshow("image", im);
		waitKey(10);


        //double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

	}

    // Stop all threads
    SLAM.Shutdown();
	waitKey(0);
	//record.release();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}


