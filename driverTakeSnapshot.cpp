#include <iostream>
#include <string>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

void callback(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cvImagePtr;
	try 
	{
		cvImagePtr = cv_bridge::toCvCopy(msg);
		cv::Mat &mat = cvImagePtr->image;

		///////////////////////////////////////

		cv::Vec3b pixel = mat.at<cv::Vec3b>(100, 500);

		std::cout << "Pixel at location 500, 100 has the following BGR values:\n";
		std::cout << "pixel[0] = " << (int)pixel[0] << "\n";
		std::cout << "pixel[1] = " << (int)pixel[1] << "\n";
		std::cout << "pixel[2] = " << (int)pixel[2] << "\n";

		cv::Mat imageHSV;

		cv::imwrite("cameraImage.png", mat);

		cv::cvtColor(mat, imageHSV, CV_BGR2HSV);

		cv::Vec3b pixelHSV = imageHSV.at<cv::Vec3b>(100, 500);

		std::cout << "Pixel at location 500, 100 has the following HSV values:\n";
		std::cout << "pixel[0] = " << (int)pixelHSV[0] << "\n";
		std::cout << "pixel[1] = " << (int)pixelHSV[1] << "\n";
		std::cout << "pixel[2] = " << (int)pixelHSV[2] << "\n";

		cv::Vec3b pixel2HSV = imageHSV.at<cv::Vec3b>(100, 100);

		std::cout << "Pixel at location 100, 100 has the following HSV values:\n";
		std::cout << "pixel[0] = " << (int)pixel2HSV[0] << "\n";
		std::cout << "pixel[1] = " << (int)pixel2HSV[1] << "\n";
		std::cout << "pixel[2] = " << (int)pixel2HSV[2] << "\n";

		cv::Vec3b pixel3HSV = imageHSV.at<cv::Vec3b>(300, 500);

		std::cout << "Pixel at location 500, 300 has the following HSV values:\n";
		std::cout << "pixel[0] = " << (int)pixel3HSV[0] << "\n";
		std::cout << "pixel[1] = " << (int)pixel3HSV[1] << "\n";
		std::cout << "pixel[2] = " << (int)pixel3HSV[2] << "\n";

		cv::Vec3b pixel4HSV = imageHSV.at<cv::Vec3b>(350, 100);

		std::cout << "Pixel at location 100, 350 has the following HSV values:\n";
		std::cout << "pixel[0] = " << (int)pixel4HSV[0] << "\n";
		std::cout << "pixel[1] = " << (int)pixel4HSV[1] << "\n";
		std::cout << "pixel[2] = " << (int)pixel4HSV[2] << "\n";

		cv::Vec3b pixel5HSV = imageHSV.at<cv::Vec3b>(350, 300);

		std::cout << "Pixel at location 300, 350 has the following HSV values:\n";
		std::cout << "pixel[0] = " << (int)pixel5HSV[0] << "\n";
		std::cout << "pixel[1] = " << (int)pixel5HSV[1] << "\n";
		std::cout << "pixel[2] = " << (int)pixel5HSV[2] << "\n";

		//////////////////////////////////////
		//DETECTING A SINGLE RED PIXEL	


		int lowArr[5] = { 0, 15, 38, 75, 130 };
		int highArr[5] = { 15, 30, 70, 120, 179 };

		for (int i = 0; i < 5; i++)
		{
			cv::Mat imageFilter;
			cv::inRange(imageHSV, cv::Scalar(lowArr[i], 20, 20), cv::Scalar(highArr[i], 255, 255), imageFilter);
			char numstr[21];
			sprintf(numstr, "result%d.png", i);
			std::string fileName = numstr;
			cv::imwrite(fileName, imageFilter);


		}

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
