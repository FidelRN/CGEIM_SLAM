#include<iostream>
#include<ros/ros.h>
#include<rosbag/bag.h>
#include<rosbag/view.h>
#include<sensor_msgs/Image.h>
#include<std_msgs/Time.h>
#include<std_msgs/Header.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>
#include <iostream>

#include "Thirdparty/DLib/FileFunctions.h"


using namespace cv;
using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "CamToSlam");


    ros::start();

    ros::Time t = ros::Time::now();

    cv::VideoCapture vcap;
    cv::Mat image;

    const std::string videoStreamAddress = "http://localhost:5555/video";
       //open the video stream and make sure it's opened
    if(!vcap.open(videoStreamAddress)) {
        std::cerr << "Error opening video stream or file" << std::endl;
        return -1;
    }

    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<sensor_msgs::Image>("/camera/image_raw", 1);

    for(;;)
    {
        if(!ros::ok())
            break;

        vcap.read(image);
        // Size: https://www.ezs3.com/public/196.cfm
        resize(image, image, Size(768, 432), 0, 0, INTER_CUBIC);
        
        cv_bridge::CvImage cvImage;
        cvImage.image = image;
        cvImage.encoding = sensor_msgs::image_encodings::RGB8;
        cvImage.header.stamp = t;

        // Publish to topic the image
        pub.publish(cvImage.toImageMsg());
        cv::waitKey(30);
    }
    ros::shutdown();

    return 0;
}