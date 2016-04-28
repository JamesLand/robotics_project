#include <iostream>
#include <string>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

const int LANDMARK_VGO = 1;
const int LANDMARK_RBG = 2; 
const int LANDMARK_OVB = 3;

std::vector<cv::KeyPoint> getBlobs(cv::Mat threshold){

	//Define parameters for blob detection
	cv::SimpleBlobDetector::Params p;

	//????????????????
	p.thresholdStep = 1;
	p.minThreshold = 0;
	p.maxThreshold = 256;
	
	p.filterByColor = true;
	p.blobColor = 255;

	p.filterByArea = true;
	p.minArea = 5000;
	p.maxArea = 300000000;

	p.filterByCircularity = false;
	p.filterByConvexity = false;
	p.filterByInertia = false;
	
	cv::SimpleBlobDetector bd(p);
	std::vector<cv::KeyPoint> keyPoints;

	bd.detect(threshold, keyPoints);

	std::cout << keyPoints.size() << "\n";
	return keyPoints;

}

/*
 * Using the image detected by the robot, performs the necessary landmark recognition algorithms.
 * http://www.learnopencv.com/blob-detection-using-opencv-python-c/
 * http://docs.opencv.org/trunk/d0/d7a/classcv_1_1SimpleBlobDetector.html#gsc.tab=0
 */
int detectLandmarks(cv::Mat &rgbImage)
{
	cv::Mat imageBGR;
	cv::cvtColor(rgbImage, imageBGR, CV_RGB2BGR);
	imwrite("theImage.png", imageBGR);

	//Convert code from RGB to HSV
	cv::Mat imageHSV;
	cv::cvtColor(rgbImage, imageHSV, CV_RGB2HSV);

	int sat = 100;
	int val = 50;

	//Thresholding for the five different colors:
	//Red, Orange, Green, Blue, and Violet.

	cv::Mat redThreshold;
	cv::Mat redPart1;
	cv::Mat redPart2;
	cv::inRange(imageHSV, cv::Scalar(0, sat, val), cv::Scalar(10, 255, 255), redPart1);
	cv::inRange(imageHSV, cv::Scalar(170, sat, val), cv::Scalar(180, 255, 255), redPart2);

	redThreshold = redPart1 | redPart2;

	cv::Mat orangeThreshold;
	cv::inRange(imageHSV, cv::Scalar(15, sat, val), cv::Scalar(30, 255, 255), orangeThreshold);

	cv::Mat greenThreshold;
	cv::inRange(imageHSV, cv::Scalar(38, sat, val), cv::Scalar(70, 255, 255), greenThreshold);

	cv::Mat blueThreshold;
	cv::inRange(imageHSV, cv::Scalar(75, sat, val), cv::Scalar(120, 255, 255), blueThreshold);

	cv::Mat violetThreshold;
	cv::inRange(imageHSV, cv::Scalar(130, sat, val), cv::Scalar(165, 255, 255), violetThreshold);

	//Blob detection for each type of color
	
//	cv::Mat redThreshold3channel;
//	cv::cvtColor(redThreshold, redThreshold3channel, CV_GRAY2BGR);

	std::vector<cv::KeyPoint> redKeyPoints = getBlobs(redThreshold);
	std::vector<cv::KeyPoint> orangeKeyPoints = getBlobs(orangeThreshold);
	std::vector<cv::KeyPoint> greenKeyPoints = getBlobs(greenThreshold);
	std::vector<cv::KeyPoint> blueKeyPoints = getBlobs(blueThreshold);
	std::vector<cv::KeyPoint> violetKeyPoints = getBlobs(violetThreshold);

	cv::Mat redIntermediate;
	cv::Mat redOutput;
	cv::cvtColor(redThreshold, redIntermediate, CV_GRAY2BGR);
//	cv::drawKeypoints(redIntermediate, redKeyPoints, redOutput, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	
	std::cout << redKeyPoints.size();
	for (std::vector<cv::KeyPoint>::size_type i = 0; i != redKeyPoints.size(); i++)
	{
		std::cout << redKeyPoints[i].pt.x << ", " <<redKeyPoints[i].pt.y << "\n";
		std::cout << redKeyPoints[i].size;
		cv::circle(redIntermediate, redKeyPoints[i].pt, redKeyPoints[i].size / 2, cv::Scalar(0, 0, 255));
	}
	imwrite("red.png", redIntermediate);

	cv::Mat orangeIntermediate;
	cv::Mat orangeOutput;
	cv::cvtColor(orangeThreshold, orangeIntermediate, CV_GRAY2BGR);
	cv::drawKeypoints(orangeIntermediate, orangeKeyPoints, orangeOutput, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	imwrite("orange.png", orangeOutput);

	cv::Mat greenIntermediate;
	cv::Mat greenOutput;
	cv::cvtColor(greenThreshold, greenIntermediate, CV_GRAY2BGR);
	cv::drawKeypoints(greenIntermediate, greenKeyPoints, greenOutput, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	imwrite("green.png", greenOutput);

	cv::Mat blueIntermediate;
	cv::Mat blueOutput;
	cv::cvtColor(blueThreshold, blueIntermediate, CV_GRAY2BGR);
	cv::drawKeypoints(blueIntermediate, blueKeyPoints, blueOutput, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	imwrite("blue.png", blueOutput);

	cv::Mat violetIntermediate;
	cv::Mat violetOutput;
	cv::cvtColor(violetThreshold, violetIntermediate, CV_GRAY2BGR);
	cv::drawKeypoints(violetIntermediate, violetKeyPoints, violetOutput, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	imwrite("violet.png", violetOutput);
	//Blob proximity logic - are the blobs close enough to each other, and oriented
	//in such a way as to believe that there is a landmark?
	//If so, return the appropriate integer value. Constants defined at the top.
}


void callback(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cvImagePtr;
	try 
	{
		cvImagePtr = cv_bridge::toCvCopy(msg);
		cv::Mat &mat = cvImagePtr->image;
		detectLandmarks(mat);

		///////////////////////////////////////

//		cv::Vec3b pixel = mat.at<cv::Vec3b>(100, 500);
//
//		std::cout << "Pixel at location 500, 100 has the following BGR values:\n";
//		std::cout << "pixel[0] = " << (int)pixel[0] << "\n";
//		std::cout << "pixel[1] = " << (int)pixel[1] << "\n";
//		std::cout << "pixel[2] = " << (int)pixel[2] << "\n";
//
//		cv::Mat imageHSV;
//
//		cv::imwrite("cameraImage.png", mat);
//
//		cv::cvtColor(mat, imageHSV, CV_BGR2HSV);
//
//		cv::Vec3b pixelHSV = imageHSV.at<cv::Vec3b>(100, 500);
//
//		std::cout << "Pixel at location 500, 100 has the following HSV values:\n";
//		std::cout << "pixel[0] = " << (int)pixelHSV[0] << "\n";
//		std::cout << "pixel[1] = " << (int)pixelHSV[1] << "\n";
//		std::cout << "pixel[2] = " << (int)pixelHSV[2] << "\n";
//
//		cv::Vec3b pixel2HSV = imageHSV.at<cv::Vec3b>(100, 100);
//
//		std::cout << "Pixel at location 100, 100 has the following HSV values:\n";
//		std::cout << "pixel[0] = " << (int)pixel2HSV[0] << "\n";
//		std::cout << "pixel[1] = " << (int)pixel2HSV[1] << "\n";
//		std::cout << "pixel[2] = " << (int)pixel2HSV[2] << "\n";
//
//		cv::Vec3b pixel3HSV = imageHSV.at<cv::Vec3b>(300, 500);
//
//		std::cout << "Pixel at location 500, 300 has the following HSV values:\n";
//		std::cout << "pixel[0] = " << (int)pixel3HSV[0] << "\n";
//		std::cout << "pixel[1] = " << (int)pixel3HSV[1] << "\n";
//		std::cout << "pixel[2] = " << (int)pixel3HSV[2] << "\n";
//
//		cv::Vec3b pixel4HSV = imageHSV.at<cv::Vec3b>(350, 100);
//
//		std::cout << "Pixel at location 100, 350 has the following HSV values:\n";
//		std::cout << "pixel[0] = " << (int)pixel4HSV[0] << "\n";
//		std::cout << "pixel[1] = " << (int)pixel4HSV[1] << "\n";
//		std::cout << "pixel[2] = " << (int)pixel4HSV[2] << "\n";
//
//		cv::Vec3b pixel5HSV = imageHSV.at<cv::Vec3b>(350, 300);
//
//		std::cout << "Pixel at location 300, 350 has the following HSV values:\n";
//		std::cout << "pixel[0] = " << (int)pixel5HSV[0] << "\n";
//		std::cout << "pixel[1] = " << (int)pixel5HSV[1] << "\n";
//		std::cout << "pixel[2] = " << (int)pixel5HSV[2] << "\n";
//
//		//////////////////////////////////////
//		//DETECTING A SINGLE RED PIXEL	
//
//
//		int lowArr[5] = { 0, 15, 38, 75, 130 };
//		int highArr[5] = { 15, 30, 70, 120, 179 };
//
//		for (int i = 0; i < 5; i++)
//		{
//			/*cv::Mat imageFilter;
//			cv::inRange(imageHSV, cv::Scalar(lowArr[i], 20, 20), cv::Scalar(highArr[i], 255, 255), imageFilter);
//			char numstr[21];
//			sprintf(numstr, "result%d.png", i);
//			std::string fileName = numstr;
//			cv::imwrite(fileName, imageFilter);
//*/
//
//		}

		exit(0);
	}
	catch (cv_bridge::Exception &e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "robot_driver");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_raw", 1, callback);
	ros::spin();

	//if (argc != 2)
	//{
	//	std::cout << "Usage: driver <ImageName>\n";
	//	return -1;
	//}

	//cv::Mat image;
	//image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);

	//if (!image.data)
	//{
	//	std::cout << "Failed to open or find image\n";
	//	return -1;
	//}
	//
	//std::cout << "Image successfully opened!\n";
	//
	//std::cout << image.cols << "\n";
	//std::cout << image.rows << "\n";
	//
	//cv::Vec3b pixel = image.at<cv::Vec3b>(100, 500);	

	//std::cout << "Pixel at location 500, 100 has the following BGR values:\n";
	//std::cout << "pixel[0] = " << (int)pixel[0] << "\n";
	//std::cout << "pixel[1] = " << (int)pixel[1] << "\n";
	//std::cout << "pixel[2] = " << (int)pixel[2] << "\n";
	//
	//cv::Mat imageHSV;

	//cv::cvtColor(image, imageHSV, CV_BGR2HSV);

	//cv::Vec3b pixelHSV = imageHSV.at<cv::Vec3b>(100, 500);
	//
	//std::cout << "Pixel at location 500, 100 has the following HSV values:\n";
	//std::cout << "pixel[0] = " << (int)pixelHSV[0] << "\n";
	//std::cout << "pixel[1] = " << (int)pixelHSV[1] << "\n";
	//std::cout << "pixel[2] = " << (int)pixelHSV[2] << "\n";

	//cv::Vec3b pixel2HSV = imageHSV.at<cv::Vec3b>(100, 100);

	//std::cout << "Pixel at location 100, 100 has the following HSV values:\n";
	//std::cout << "pixel[0] = " << (int)pixel2HSV[0] << "\n";
	//std::cout << "pixel[1] = " << (int)pixel2HSV[1] << "\n";
	//std::cout << "pixel[2] = " << (int)pixel2HSV[2] << "\n";

	//cv::Vec3b pixel3HSV = imageHSV.at<cv::Vec3b>(300, 500);

	//std::cout << "Pixel at location 500, 300 has the following HSV values:\n";
	//std::cout << "pixel[0] = " << (int)pixel3HSV[0] << "\n";
	//std::cout << "pixel[1] = " << (int)pixel3HSV[1] << "\n";
	//std::cout << "pixel[2] = " << (int)pixel3HSV[2] << "\n";
	//
	//cv::Vec3b pixel4HSV = imageHSV.at<cv::Vec3b>(350, 100);

	//std::cout << "Pixel at location 100, 350 has the following HSV values:\n";
	//std::cout << "pixel[0] = " << (int)pixel4HSV[0] << "\n";
	//std::cout << "pixel[1] = " << (int)pixel4HSV[1] << "\n";
	//std::cout << "pixel[2] = " << (int)pixel4HSV[2] << "\n";
	//
	//cv::Vec3b pixel5HSV = imageHSV.at<cv::Vec3b>(350, 300);

	//std::cout << "Pixel at location 300, 350 has the following HSV values:\n";
	//std::cout << "pixel[0] = " << (int)pixel5HSV[0] << "\n";
	//std::cout << "pixel[1] = " << (int)pixel5HSV[1] << "\n";
	//std::cout << "pixel[2] = " << (int)pixel5HSV[2] << "\n";
	//
	////////////////////////////////////////
	////DETECTING A SINGLE RED PIXEL	


	//int lowArr [5] = {0, 15, 38, 75, 130};
	//int highArr [5] = {15, 30, 70, 120, 179};	

	//for (int i = 0; i < 5; i++)
	//{
	//	cv::Mat imageFilter;
	//	cv::inRange(imageHSV, cv::Scalar(lowArr[i], 20, 20), cv::Scalar(highArr[i], 255, 255), imageFilter);
	//	char numstr[21];
	//	sprintf(numstr, "result%d.png", i); 
	//	std::string fileName =  numstr;
	//	cv::imwrite(fileName, imageFilter);		

	//	
	//}
	//

	return 0;

	

}
