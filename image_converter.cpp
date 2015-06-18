#include "image_converter.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Empty.h>
#include <ardrone_autonomy/Navdata.h>
#include <geometry_msgs/Twist.h>
#include <cv.h>
#include <highgui.h>
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/photo/photo.hpp"
#include "opencv2/nonfree/features2d.hpp"
#define IMAGE_PATH "/ardrone/image_raw" //Quadcopter
//#define IMAGE_PATH "/image_raw" //Webcam

namespace image_converter
{
  //image_transport::Subscriber image_sub_;
  //ros::NodeHandle nh_;
  //image_transport::ImageTransport it_;
  //image_transport::Subscriber image_sub_;
  //image_transport::Publisher image_pub_;
  
  //Mat src1;
  //int testCount;
  ImageConverter::ImageConverter()
    : it_(nh_)
  {
    testCount = 0;
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe(IMAGE_PATH, 1, 
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
  
    //cv::namedWindow(OPENCV_WINDOW);
  }

  ImageConverter::~ImageConverter()
  {
    //cv::destroyWindow(OPENCV_WINDOW);w
  }
  double data[] = { 8.0617692817080149e+02, 0., 3.1935989457565023e+02, 0.,8.0304475206552934e+02, 1.8492321512613066e+02, 0., 0., 1. };
  void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    testCount++;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    //if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //  cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
    
    
    src1 = cv_ptr->image;
    Mat temp = src1.clone();
    Mat cameraMatrix;
    Mat distCoeffs;
    FileStorage fs("calib.xml",FileStorage::READ);
    fs["cameraMatrix"] >> cameraMatrix;
    fs["distCoeffs"] >> distCoeffs;
    fs.release();
    undistort(temp, src1, cameraMatrix, distCoeffs);
    //Remove light
    //inRange(src1, Scalar( 0 , 0, 0), Scalar(180, 255, 250), temp );
    //cvtColor(temp,temp,COLOR_GRAY2BGR);
    //bitwise_and(src1,temp,src1);
    //bitwise_not ( temp, temp);
    //inpaint(src1, temp, src1, 10, INPAINT_NS);
    // Update GUI Window
    //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
}