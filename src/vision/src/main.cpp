#include "CMT.h"
#include "gui.h"
#include "image_converter.h"
#include "controller.h"

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
using namespace image_converter;
using namespace controller;
#define DT 0.05
#define X 0
#define Y 1
#define ROT 2
#define Z 3
#define X_SLIDER_WEIGHT -10
#define Y_SLIDER_WEIGHT 1000
#define Z_SLIDER_WEIGHT 100
#define ROT_SLIDER_WEIGHT 2
#define SUCCESS_RATIO 0.3

static string WIN_NAME = "CMT";

/// Matrices to store images

Mat dst;
const int slider_max = 1000;
int kpx_slider = 0;
int kdx_slider = 0;
int kpy_slider = 0;
int kdy_slider = 0;
int kpz_slider = 0;
int kdz_slider = 0;
int kpr_slider = 0;
int kdr_slider = 0;
bool controller_updated = false;



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

static void on_mouse( int event, int x, int y, int, void* )
{
    if(event == EVENT_RBUTTONDOWN) {
        Controller reg;
        reg.auto_hover();
        reg.land();
        throw exception();
        return;
    }
    if( event != EVENT_LBUTTONDOWN )
        return;
}

void loadTarget(Mat* im, Rect* rect){
    FileStorage fs("target.yml", FileStorage::READ);
    fs["im"] >> (*im);
    fs["rect"] >> (*rect);
    fs.release();
}

void saveTarget(Mat im, Rect rect){
    FileStorage fs("target.yml", FileStorage::WRITE);
    fs << "im" << im;
    fs << "rect" << rect;
    fs.release();
}

Rect setTarget(ImageConverter* ic, Controller* reg, CMT* cmt, Mat* im0, VideoWriter* vid, ros::Rate r){
    (*cmt) = CMT();
    bool show_preview = true;
    Rect rect;
    while (show_preview)
    {
        ros::spinOnce();
        Mat preview;
        //cap >> preview;
        preview = ic->src1;
        screenLog(preview, "Press a key to start selecting an object.");
        imshow(WIN_NAME, preview);
        //ROS_INFO("Count: %d",ic.testCount);
        vid->write(preview);
        char k = waitKey(10);
        if (k != -1) {
            show_preview = false;
        }
        r.sleep();
    }

    //Get initial image
    //cap >> im0;
    (*im0) = ic->src1.clone();
    //Get bounding box from user
    rect = getRect((*im0), WIN_NAME);
    reg->setTargetRect(rect);
    FILE_LOG(logINFO) << "Using " << rect.x << "," << rect.y << "," << rect.width << "," << rect.height
        << " as initial bounding box.";

    //Convert im0 to grayscale
    Mat im0_gray;
    cvtColor((*im0), im0_gray, CV_BGR2GRAY);

    //Initialize CMT
    cmt->initialize(im0_gray, rect);
    setMouseCallback(WIN_NAME, on_mouse,0);
    saveTarget(im0_gray,rect);
    return rect;
}

void loadPID(Controller * reg){
    reg->loadController();
    kpx_slider = reg->Kp[X]*X_SLIDER_WEIGHT*slider_max;
    kdx_slider = reg->Kd[X]*X_SLIDER_WEIGHT*slider_max;
    kpy_slider = reg->Kp[Y]*Y_SLIDER_WEIGHT*slider_max;
    kdy_slider = reg->Kd[Y]*Y_SLIDER_WEIGHT*slider_max;
    kpz_slider = reg->Kp[Z]*Z_SLIDER_WEIGHT*slider_max;
    kdz_slider = reg->Kd[Z]*Z_SLIDER_WEIGHT*slider_max;
    kpr_slider = reg->Kp[ROT]*ROT_SLIDER_WEIGHT*slider_max;
    kdr_slider = reg->Kd[ROT]*ROT_SLIDER_WEIGHT*slider_max;
}

void on_trackbar1( int, void* )
{
    controller_updated = true;
}
void on_trackbar2( int, void* )
{
    controller_updated = true;
}
void on_trackbar3( int, void* )
{
    controller_updated = true;
}
void on_trackbar4( int, void* )
{
    controller_updated = true;
}
void on_trackbar5( int, void* )
{
    controller_updated = true;
}
void on_trackbar6( int, void* )
{
    controller_updated = true;
}
void on_trackbar7( int, void* )
{
    controller_updated = true;
}
void on_trackbar8( int, void* )
{
    controller_updated = true;
}
void drawText(Mat img, string text,int c){
    int fontFace = FONT_HERSHEY_SIMPLEX;
    double fontScale = 1;
    int thickness = 3;
    
    // center the text
    Point textOrg(0,320);
    
    // then put the text itself
    if (c == 0)
    {
        putText(img, text, textOrg, fontFace, fontScale,
            Scalar(0,255,0), thickness, 8);
    }
    else{
        putText(img, text, textOrg, fontFace, fontScale,
            Scalar(0,0,255), thickness, 8);
    }
}
bool last_flag = false;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "test");
    ImageConverter ic;
    ic.loadCalibration();
    Controller reg;
    ros::Rate r(1/DT); // 10 hz
    ros::spinOnce();
    reg.calibrate();
    //Create a CMT object
    reg.wait(1.0);

    CMT cmt;

    //Initialization bounding box
    Rect rect;

    //Parse args
    int verbose_flag = 0;
    string input_path;

    const int detector_cmd = 1000;
    const int descriptor_cmd = 1001;
    const int no_scale_cmd = 1003;
    const int with_rotation_cmd = 1004;
    const int remember_last = 1005;

    struct option longopts[] =
    {
        //No-argument options
        {"verbose", no_argument, &verbose_flag, 1},
        //Argument options
        {"detector", required_argument, 0, detector_cmd},
        {"descriptor", required_argument, 0, descriptor_cmd},
        {"no-scale", no_argument, 0, no_scale_cmd},
        {"with-rotation", no_argument, 0, with_rotation_cmd},
        {"remember-last", no_argument, 0, remember_last},
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
            case remember_last:
                last_flag = true;
                break;
            case '?':
                return 1;
        }

    }

    //Set up logging
    FILELog::ReportingLevel() = verbose_flag ? logDEBUG : logINFO;
    Output2FILE::Stream() = stdout; //Log to stdout

    //Normal mode

    //Create window
    namedWindow(WIN_NAME);
    setMouseCallback(WIN_NAME, on_mouse,0);

    //Reset quadcopter
    reg.init();
    //Takeoff
    reg.takeoff();
    //reg.elevate(1000);
    reg.auto_hover();
    reg.setTargetRot();

    //Show preview until key is pressed
    Mat im0;
    VideoWriter vid;
    vid.open("test.avi",CV_FOURCC('U', '2', '6', '3'),1/DT,Size(640,360));
    if (last_flag)
    {
        Mat im0_gray;
        Rect rect;
        loadTarget(&im0_gray,&rect);
        reg.setTargetRect(rect);
        cmt.initialize(im0_gray, rect);
        setMouseCallback(WIN_NAME, on_mouse,0);
    }
    else
    {
        setTarget(&ic, &reg, &cmt, &im0, &vid, r);
    }
    int frame = 0;

    //Read PID values
    loadPID(&reg);
    
    //Create trackbars
    createTrackbar( "Kp x: ", WIN_NAME, &kpx_slider, slider_max, on_trackbar1 );
    createTrackbar( "Kd x: ", WIN_NAME, &kdx_slider, slider_max, on_trackbar2 );
    createTrackbar( "Kp y: ", WIN_NAME, &kpy_slider, slider_max, on_trackbar3 );
    createTrackbar( "Kd y: ", WIN_NAME, &kdy_slider, slider_max, on_trackbar4 );
    createTrackbar( "Kp z: ", WIN_NAME, &kpz_slider, slider_max, on_trackbar5 );
    createTrackbar( "Kd z: ", WIN_NAME, &kdz_slider, slider_max, on_trackbar6 );
    createTrackbar( "Kp r: ", WIN_NAME, &kpr_slider, slider_max, on_trackbar7 );
    createTrackbar( "Kd r: ", WIN_NAME, &kdr_slider, slider_max, on_trackbar8 );
    //Main loop
    while (ros::ok()) {
    	ros::spinOnce();
        frame++;
        Mat im;
        if (controller_updated)
        {
            reg.Kp[X] = (double)(kpx_slider)/(X_SLIDER_WEIGHT*slider_max);
            reg.Kd[X] = (double)(kdx_slider)/(X_SLIDER_WEIGHT*slider_max);
            reg.Kp[Y] = (double)(kpy_slider)/(Y_SLIDER_WEIGHT*slider_max);
            reg.Kd[Y] = (double)(kdy_slider)/(Y_SLIDER_WEIGHT*slider_max);
            reg.Kp[Z] = (double)(kpz_slider)/(Z_SLIDER_WEIGHT*slider_max);
            reg.Kd[Z] = (double)(kdz_slider)/(Z_SLIDER_WEIGHT*slider_max);
            reg.Kp[ROT] = (double)(kpr_slider)/(ROT_SLIDER_WEIGHT*slider_max);
            reg.Kd[ROT] = (double)(kdr_slider)/(ROT_SLIDER_WEIGHT*slider_max);
            reg.saveController();
        }
        
        im = ic.src1;

        Mat im_gray;
        cvtColor(im, im_gray, CV_BGR2GRAY);

        //Let CMT process the frame
        Rect box = cmt.processFrame(im_gray);
        Point2f center = Point2f(box.x + box.width/2.0, box.y + box.height/2.0);
        circle( im, center, 5, Scalar(0,0,255), -1, 8, 0 );
        line(im, Point(0,180),Point(640,180),Scalar(0,255,0),1,8,0);
        line(im, Point(320,0),Point(320,360),Scalar(0,255,0),1,8,0);
        reg.update_state(center, box);
        if(cmt.ratio > SUCCESS_RATIO) {
            drawText(im,"Target lockon",0);
            reg.control(DT);
        } else {
            drawText(im,"Target lost",1);
            reg.auto_hover();
        }

        char key = display(im, cmt);
        vid.write(im);
        if(key == 'q') break;
        else if (key == 'k'){
            reg.auto_hover();
            setTarget(&ic, &reg, &cmt, &im0, &vid, r);
            continue;
        }else if (key == 'c'){
            imwrite("im.png",im);
        }
        reg.logData();
        //TODO: Provide meaningful output
        FILE_LOG(logINFO) << "#" << frame << " active: " << cmt.points_active.size();
        FILE_LOG(logINFO) << "#" << frame << " total: " << cmt.points_total;
        FILE_LOG(logINFO) << "#" << frame << " ratio: " << cmt.ratio;
        ROS_INFO("Target: %g",reg.target[Z]);
        r.sleep();
    }
    reg.land();

    return 0;
}
