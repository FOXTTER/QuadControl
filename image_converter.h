#ifndef IMAGE_CONVERTER_H

#define IMAGE_CONVERTER_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Empty.h>
#include <ardrone_autonomy/Navdata.h>
#include <geometry_msgs/Twist.h>
#include <cv.h>
#include <highgui.h>

using namespace cv;
using namespace std;

namespace image_converter
{
class ImageConverter
{  
public:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  Mat src1;
  int testCount;
  ImageConverter();

  ~ImageConverter();

  void imageCb(const sensor_msgs::ImageConstPtr& msg);
};
}
#endif