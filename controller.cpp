#include "controller.h"
#include <stdio.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Empty.h>
#include <ardrone_autonomy/Navdata.h>
#include <geometry_msgs/Twist.h>
#include <cv.h>
#include <highgui.h>
//PID values in case a predefined controller is selected
#define KP_X -0.16
#define KP_Y 0.00016
#define KP_ROT 0.5
#define KP_Z 0.001
#define KI_X 0.000
#define KI_Y 0.0000
#define KI_ROT 0.0
#define KI_Z 0.0000
#define KD_X -2
#define KD_Y 0.000075
#define KD_ROT 3
#define KD_Z 1
#define X 0
#define Y 1
#define ROT 2
#define Z 3
#define LANDED 2

#define PIXEL_DIST_X 180 //pixels
#define PIXEL_DIST_Y 320
#define FOCAL_LENGTH_X 607.7
#define FOCAL_LENGTH_Y 554

#define FILTER_WEIGHT 0.5
using namespace cv;




namespace controller
{

	void Controller::nav_callback(const ardrone_autonomy::Navdata& msg_in)
	{
		//Take in navdata from ardrone
		msg_in_global = msg_in;
	}
	void Controller::logData()
	{
	  FILE* pFile = fopen("quadlog.txt", "a");
	  fprintf(pFile, "%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g\n",(double)ros::Time::now().toSec()-start_time,measured[X],measured[Y],measured[Z],measured[ROT],
      output[X],output[Y],output[Z],output[ROT],error[X],error[Y],error[Z],error[ROT]);
	  //fprintf(pFile,"%g,%g,%g,%g,%g\n",(double)ros::Time::now().toSec()-start_time,measuredRaw[Y],measured[Y],measuredRaw[Z],measuredRaw[Z]);
    fclose(pFile);
	}

	Controller::Controller()
	:previous_error(4,0.0)
	,error(4,0.0)
	,output(4,0.0)
	,derivative(4,0.0)
	,integral(4,0.0)
	,target(4,0.0)
	,Kp(4,0.0)
	,Ki(4,0.0)
	,Kd(4,0.0)
	,measured(4,0.0)
  ,measuredRaw(4,0.0)
	{
    //In order to use predefine PID values uncomment this
		//Kp[X] = KP_X;
		//Kp[Y] = KP_Y;
		//Kp[ROT] = KP_ROT;
		//Kp[Z] = KP_Z;
		//Ki[X] = KI_X;
		//Ki[Y] = KI_Y;
		//Ki[ROT] = KI_ROT;
		//Ki[Z] = KI_Z;
		nav_sub = node.subscribe("/ardrone/navdata", 1, &Controller::nav_callback,this);
		pub_twist = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1); /* Message queue length is just 1 */
		pub_empty_takeoff = node.advertise<std_msgs::Empty>("/ardrone/takeoff", 1); /* Message queue length is just 1 */
		pub_empty_land = node.advertise<std_msgs::Empty>("/ardrone/land", 1); /* Message queue length is just 1 */
		pub_empty_reset = node.advertise<std_msgs::Empty>("/ardrone/reset", 1); /* Message queue length is just 1 */
	}


	void Controller::takeoff(){
		pub_empty_takeoff.publish(emp_msg);
	}
	void Controller::calibrate(){
		//Controller::Foo foo;
		//ros::service::call("/ardrone/flattrim",foo);
	}
	void Controller::wait(double tid){
		double time_start=(double)ros::Time::now().toSec();
 		while (ros::ok() && ((double)ros::Time::now().toSec()< time_start+tid)){
 			ros::spinOnce();
 		}
	}

  void Controller::saveController(){
    FileStorage fs("PIDvals.yml", FileStorage::WRITE);
    fs << "Kpx" << Kp[X];
    fs << "Kdx" << Kd[X];
    fs << "Kpy" << Kp[Y];
    fs << "Kdy" << Kd[Y];
    fs << "Kpz" << Kp[Z];
    fs << "Kdz" << Kd[Z];
    fs << "Kpr" << Kp[ROT];
    fs << "Kdr" << Kd[ROT];
    fs.release();
  }

  void Controller::loadController(){
    FileStorage fs("PIDvals.yml", FileStorage::READ);
    fs["Kpx"] >> Kp[X];
    fs["Kdx"] >> Kd[X];
    fs["Kpy"] >> Kp[Y];
    fs["Kdy"] >> Kd[Y];
    fs["Kpz"] >> Kp[Z];
    fs["Kdz"] >> Kd[Z];
    fs["Kpr"] >> Kp[ROT];
    fs["Kdr"] >> Kd[ROT];
    fs.release();
  }

  void Controller::land(){
      pub_empty_land.publish(emp_msg);
  }
  
  void Controller::reset(){
      pub_empty_reset.publish(emp_msg);
  }

  void Controller::init(){
      while (msg_in_global.state != LANDED) {
      	ros::spinOnce();
          Controller::reset();
          ROS_INFO("State: %d",msg_in_global.state);
      }
      start_time = (double)ros::Time::now().toSec();
  }
  //Altitude in millimeters
  void Controller::elevate(double altitude){ 
  	twist_msg.linear.z = 0.5;
  	pub_twist.publish(twist_msg);
  	while(msg_in_global.altd < altitude){
  		ros::spinOnce();
  	}
  	twist_msg.linear.z = 0;
  	Controller::auto_hover();
  }

  void Controller::setTargetRot(){
  	target[ROT] = 0;
  }
  void Controller::setTargetRect(Rect rect){
  	target[X] = sqrt((PIXEL_DIST_X*PIXEL_DIST_Y)/(rect.width*rect.height));
  }

  void Controller::auto_hover(){
  	pub_twist.publish(twist_msg_hover);
  }

  //Filtered data
	void Controller::update_state(Point2f center, Rect rect)
	{
		//Kun til front kamera
		measured[ROT] = atan(((center.x-PIXEL_DIST_Y)*tan(1.047/2))/(PIXEL_DIST_Y/2));
		measured[X] = sqrt((PIXEL_DIST_X*PIXEL_DIST_Y)/(rect.width*rect.height));
		measured[Y] = measured[Y]+ FILTER_WEIGHT*((center.x-PIXEL_DIST_Y)-measured[Y]);
		measured[Z] = measured[Z]+ FILTER_WEIGHT*(((center.y-PIXEL_DIST_X)+sin(msg_in_global.rotY*3.14/180)*FOCAL_LENGTH_X)-measured[Z]); 
		measuredRaw[Y] = center.x-PIXEL_DIST_Y;
    measuredRaw[Z] = (center.y-PIXEL_DIST_X)+sin(msg_in_global.rotY*3.14/180)*FOCAL_LENGTH_X;
	}

	void Controller::control(double dt)
	 {	//Original controller
	 	for(int i = 0; i < 4; i++)
	 	{
      	error[i] = target[i] - measured[i];
      	if (output[i] < 1 && output[i] > -1)
      	{
      		integral[i] = integral[i] + error[i] * dt;
      	}
      	derivative[i] = (error[i]-previous_error[i])/dt;
      	output[i] = Kp[i]*error[i] + Ki[i] * integral[i] + Kd[i] * derivative[i];
      	previous_error[i] = error[i];
    }

    twist_msg.angular.z= output[ROT];
    twist_msg.linear.x = output[X];
    twist_msg.linear.y = output[Y];
    twist_msg.linear.z = output[Z];

    pub_twist.publish(twist_msg);

    ROS_INFO("Measured pos = (%g,%g)",measured[X],measured[Y]);
    ROS_INFO("Output x: %g", output[X]);
  	ROS_INFO("Output y: %g", output[Y]);
  	ROS_INFO("Output z: %g", output[Z]);
  	ROS_INFO("Output r: %g", output[ROT]);
	 }
}