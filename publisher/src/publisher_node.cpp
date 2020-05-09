#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/version.hpp>
#include <opencv2/core/core.hpp>
#include <string>
#include <sstream>
#include <iostream>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <highgui.h>
#include <math.h>



cv::Mat src_raw;
cv::Mat src_in;
cv::Mat src_out;
cv::Mat src_blur;
cv::Mat src_gray;
cv::Mat3f circles;

cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
cv_bridge::CvImagePtr cv_ptr2(new cv_bridge::CvImage);


void imageCallback(const sensor_msgs::ImageConstPtr& src_img)
{
  
  //ROS_INFO("Image_Msg Start");
  try
  {
    cv_ptr  = cv_bridge::toCvCopy(src_img, sensor_msgs::image_encodings::BGR8);
    cv_ptr2 = cv_bridge::toCvCopy(src_img, sensor_msgs::image_encodings::BGR8);
    
    src_in  = cv_ptr->image;
    src_raw = cv_ptr2->image;
  }
  catch (cv_bridge::Exception& e)
  {
    return;
  }


  
  // strel_size is the size of the structuring element
  
  cv::Size strel_size;
  strel_size.width = 20;
  strel_size.height = 20;
  // Create an elliptical structuring element
  cv::Mat strel = cv::getStructuringElement(cv::MORPH_ELLIPSE,strel_size);
  // Apply an opening morphological operation using the structing element to the image twice
  cv::morphologyEx(src_in, src_in ,cv::MORPH_OPEN,strel, cv::Point(-1, -1),2);
  // Convert White on Black to Black on White by inverting the image
  //cv::bitwise_not(src_in, src_in);
  
  
  cv::GaussianBlur(src_in, src_blur, cv::Size(7, 7), 2, 2);
  cv::cvtColor(src_blur, src_out, CV_RGB2GRAY);
  
  circles.rows = 0;
  cv::HoughCircles(src_out, circles, CV_HOUGH_GRADIENT, 1, 100, 80, 25, 20, 60);

  int x, y = 0;
  for(int i = 0; i < circles.rows; i++)
  {
    cv::Point center(cvRound(circles(i)(0)), cvRound(circles(i)(1)));
    x = center.x;
    y = center.y;

    int radius = cvRound(circles(i)(2));
    cv::circle(src_raw, center, radius, CV_RGB(255, 0, 0), 3);
    cv::circle(src_raw, center, 3, CV_RGB(0, 0, 255), -1);

  }
  ROS_INFO("circles: [%d] -> %d, %d", circles.rows, x, y);
 
  
  cv_ptr->encoding = "bgr8"; 
  cv_ptr->image = src_raw;
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "circle_detecter");
  ros::NodeHandle n;
  ros::MultiThreadedSpinner spinner(2);
  ros::Rate loop_rate(500);
  image_transport::ImageTransport cam(n);
  image_transport::Subscriber sub = cam.subscribe("/usb_cam/image_raw", 1, imageCallback);
  image_transport::Publisher pub_ = cam.advertise("/image_output/circle_detect", 1);
  
  
  
  ROS_INFO("------- start -------");
    
  while(n.ok())
  {
    pub_.publish(cv_ptr->toImageMsg());
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
