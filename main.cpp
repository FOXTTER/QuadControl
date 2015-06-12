#include "CMT.h"
#include "gui.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <fstream>
#include <cstdio>

//Vores libraries
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Empty.h>
#include <ardrone_autonomy/Navdata.h>
#include <geometry_msgs/Twist.h>
#include <cv.h>
#include <highgui.h>
//#include <stdio.h>
#include <string>
#include <math.h>
//#define IMAGE_PATH "/ardrone/image_raw" //Quadcopter
#define IMAGE_PATH "/image_raw" //Webcam

//Slut

#ifdef __GNUC__
#include <getopt.h>
#else
#include "getopt/getopt.h"
#endif

using cmt::CMT;
using cv::imread;
using cv::namedWindow;
using cv::Scalar;
using cv::VideoCapture;
using cv::waitKey;
using std::cerr;
using std::istream;
using std::ifstream;
using std::stringstream;
using std::ofstream;
using std::cout;
using std::min_element;
using std::max_element;
using std::endl;
using ::atof;
using namespace cv;
using namespace std;

static string WIN_NAME = "CMT";

/// Matrices to store images
Mat src1;
Mat dst;
image_transport::Subscriber image_sub_;
class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:

  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe(IMAGE_PATH, 1, 
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
  
    //cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    //cv::destroyWindow(OPENCV_WINDOW);w
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
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
    
    if (true)
    {
        src1 = cv_ptr->image;
    }

    // Update GUI Window
    //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};



vector<float> getNextLineAndSplitIntoFloats(istream& str)
{
    vector<float>   result;
    string                line;
    getline(str,line);

    stringstream          lineStream(line);
    string                cell;
    while(getline(lineStream,cell,','))
    {
        result.push_back(atof(cell.c_str()));
    }
    return result;
}

int display(Mat im, CMT & cmt)
{
    //Visualize the output
    //It is ok to draw on im itself, as CMT only uses the grayscale image
    for(size_t i = 0; i < cmt.points_active.size(); i++)
    {
        circle(im, cmt.points_active[i], 2, Scalar(255,0,0));
    }

    Point2f vertices[4];
    cmt.bb_rot.points(vertices);
    for (int i = 0; i < 4; i++)
    {
        line(im, vertices[i], vertices[(i+1)%4], Scalar(255,0,0));
    }

    imshow(WIN_NAME, im);

    return waitKey(5);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "test");
    ImageConverter ic;
    ros::Rate r(30); // 10 hz
    ros::spinOnce();
    //Create a CMT object

    double time_start=(double)ros::Time::now().toSec();
 	while (ros::ok() && ((double)ros::Time::now().toSec()< time_start+0.1)){
 		ros::spinOnce();
 	}

    CMT cmt;

    //Initialization bounding box
    Rect rect;

    //Parse args
    int challenge_flag = 0;
    int loop_flag = 0;
    int verbose_flag = 0;
    int bbox_flag = 0;
    string input_path;

    const int detector_cmd = 1000;
    const int descriptor_cmd = 1001;
    const int bbox_cmd = 1002;
    const int no_scale_cmd = 1003;
    const int with_rotation_cmd = 1004;

    struct option longopts[] =
    {
        //No-argument options
        {"challenge", no_argument, &challenge_flag, 1},
        {"loop", no_argument, &loop_flag, 1},
        {"verbose", no_argument, &verbose_flag, 1},
        //Argument options
        {"bbox", required_argument, 0, bbox_cmd},
        {"detector", required_argument, 0, detector_cmd},
        {"descriptor", required_argument, 0, descriptor_cmd},
        {"no-scale", no_argument, 0, no_scale_cmd},
        {"with-rotation", no_argument, 0, with_rotation_cmd},
        {0, 0, 0, 0}
    };

    int index = 0;
    int c;
    while((c = getopt_long(argc, argv, "v", longopts, &index)) != -1)
    {
        switch (c)
        {
            case 'v':
                verbose_flag = true;
                break;
            case bbox_cmd:
                {
                    //TODO: The following also accepts strings of the form %f,%f,%f,%fxyz...
                    string bbox_format = "%f,%f,%f,%f";
                    float x,y,w,h;
                    int ret = sscanf(optarg, bbox_format.c_str(), &x, &y, &w, &h);
                    if (ret != 4)
                    {
                        cerr << "bounding box must be given in format " << bbox_format << endl;
                        return 1;
                    }

                    bbox_flag = 1;
                    rect = Rect(x,y,w,h);
                }
                break;
            case detector_cmd:
                cmt.str_detector = optarg;
                break;
            case descriptor_cmd:
                cmt.str_descriptor = optarg;
                break;
            case no_scale_cmd:
                cmt.consensus.estimate_scale = false;
                break;
            case with_rotation_cmd:
                cmt.consensus.estimate_rotation = true;
                break;
            case '?':
                return 1;
        }

    }

    //One argument remains
    if (optind == argc - 1)
    {
        input_path = argv[optind];
    }

    else if (optind < argc - 1)
    {
        cerr << "Only one argument is allowed." << endl;
        return 1;
    }

    //Set up logging
    FILELog::ReportingLevel() = verbose_flag ? logDEBUG : logINFO;
    Output2FILE::Stream() = stdout; //Log to stdout

    //Challenge mode
    if (challenge_flag)
    {
        //Read list of images
        ifstream im_file("images.txt");
        vector<string> files;
        string line;
        while(getline(im_file, line ))
        {
            files.push_back(line);
        }

        //Read region
        ifstream region_file("region.txt");
        vector<float> coords = getNextLineAndSplitIntoFloats(region_file);

        if (coords.size() == 4) {
            rect = Rect(coords[0], coords[1], coords[2], coords[3]);
        }

        else if (coords.size() == 8)
        {
            //Split into x and y coordinates
            vector<float> xcoords;
            vector<float> ycoords;

            for (size_t i = 0; i < coords.size(); i++)
            {
                if (i % 2 == 0) xcoords.push_back(coords[i]);
                else ycoords.push_back(coords[i]);
            }

            float xmin = *min_element(xcoords.begin(), xcoords.end());
            float xmax = *max_element(xcoords.begin(), xcoords.end());
            float ymin = *min_element(ycoords.begin(), ycoords.end());
            float ymax = *max_element(ycoords.begin(), ycoords.end());

            rect = Rect(xmin, ymin, xmax-xmin, ymax-ymin);
            cout << "Found bounding box" << xmin << " " << ymin << " " <<  xmax-xmin << " " << ymax-ymin << endl;
        }

        else {
            cerr << "Invalid Bounding box format" << endl;
            return 0;
        }

        //Read first image
        Mat im0 = imread(files[0]);
        Mat im0_gray;
        cvtColor(im0, im0_gray, CV_BGR2GRAY);

        //Initialize cmt
        cmt.initialize(im0_gray, rect);

        //Write init region to output file
        ofstream output_file("output.txt");
        output_file << rect.x << ',' << rect.y << ',' << rect.width << ',' << rect.height << std::endl;

        //Process images, write output to file
        for (size_t i = 1; i < files.size(); i++)
        {
            FILE_LOG(logINFO) << "Processing frame " << i << "/" << files.size();
            Mat im = imread(files[i]);
            Mat im_gray;
            cvtColor(im, im_gray, CV_BGR2GRAY);
            cmt.processFrame(im_gray);
            if (verbose_flag)
            {
                display(im, cmt);
            }
            rect = cmt.bb_rot.boundingRect();
            output_file << rect.x << ',' << rect.y << ',' << rect.width << ',' << rect.height << std::endl;
        }

        output_file.close();

        return 0;
    }

    //Normal mode

    //Create window
    namedWindow(WIN_NAME);

    VideoCapture cap;

    bool show_preview = true;

    /*//If no input was specified
    if (input_path.length() == 0)
    {
        cap.open(0); //Open default camera device
    }

    //Else open the video specified by input_path
    else
    {
        cap.open(input_path);
        show_preview = false;
    }

    //If it doesn't work, stop
    if(!cap.isOpened())
    {
        cerr << "Unable to open video capture." << endl;
        return -1;
    }*/

    //Show preview until key is pressed
    while (show_preview)
    {
        ros::spinOnce();
        Mat preview;
        //cap >> preview;
        preview = src1;
        screenLog(preview, "Press a key to start selecting an object.");
        imshow(WIN_NAME, preview);

        char k = waitKey(10);
        if (k != -1) {
            show_preview = false;
        }
    }

    //Get initial image
    Mat im0;
    //cap >> im0;
    im0 = src1.clone();
    //If no bounding was specified, get it from user
    if (!bbox_flag)
    {
        rect = getRect(im0, WIN_NAME);
    }

    FILE_LOG(logINFO) << "Using " << rect.x << "," << rect.y << "," << rect.width << "," << rect.height
        << " as initial bounding box.";

    //Convert im0 to grayscale
    Mat im0_gray;
    cvtColor(im0, im0_gray, CV_BGR2GRAY);

    //Initialize CMT
    cmt.initialize(im0_gray, rect);

    int frame = 0;

    //Main loop
    while (ros::ok())
    {
    	ros::spinOnce();
        frame++;

        Mat im;

        //If loop flag is set, reuse initial image (for debugging purposes)
        if (loop_flag) im0.copyTo(im);
        else im = src1;//cap >> im; //Else use next image in stream

        Mat im_gray;
        cvtColor(im, im_gray, CV_BGR2GRAY);

        //Let CMT process the frame
        Point2f center = cmt.processFrame(im_gray);
        circle( im, center, 5, Scalar(0,0,255), -1, 8, 0 );

        char key = display(im, cmt);

        if(key == 'q') break;

        //TODO: Provide meaningful output
        FILE_LOG(logINFO) << "#" << frame << " active: " << cmt.points_active.size();
        r.sleep();
    }

    return 0;
}
