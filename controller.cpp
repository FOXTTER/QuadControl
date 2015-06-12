#include "controller.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Empty.h>
#include <ardrone_autonomy/Navdata.h>
#include <geometry_msgs/Twist.h>
#include <cv.h>
#include <highgui.h>
#define KP_X 0.05
#define KP_Y 0.05
#define KP_ROT 0.0
#define KP_Z 0
#define X 0
#define Y 1
#define ROT 2
#define Z 3

//Husk at dobbelttjek disse værdier
#define GAMMA_X 40 //grader
#define GAMMA_Y 64
#define PIXEL_DIST_X 180 //pixels
#define PIXEL_DIST_Y 320
using namespace cv;




namespace controller
{
	void Controller::nav_callback(const ardrone_autonomy::Navdata& msg_in)
	{
		//Take in navdata from ardrone
		msg_in_global = msg_in;
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
	{
		Kp[X] = KP_X;
		Kp[Y] = KP_Y;
		Kp[ROT] = KP_ROT;
		Kp[Z] = KP_Z;
		nav_sub = node.subscribe("/ardrone/navdata", 1, &Controller::nav_callback,this);
		pub_twist = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1); /* Message queue length is just 1 */
		pub_empty_takeoff = node.advertise<std_msgs::Empty>("/ardrone/takeoff", 1); /* Message queue length is just 1 */
		pub_empty_land = node.advertise<std_msgs::Empty>("/ardrone/land", 1); /* Message queue length is just 1 */
		pub_empty_reset = node.advertise<std_msgs::Empty>("/ardrone/reset", 1); /* Message queue length is just 1 */
	}


	void Controller::takeoff(){
		pub_empty_takeoff.publish(emp_msg);
	}
	void Controller::wait(double tid){
		double time_start=(double)ros::Time::now().toSec();
 		while (ros::ok() && ((double)ros::Time::now().toSec()< time_start+tid)){
 			ros::spinOnce();
 		}
	}
    void Controller::land(){
        pub_empty_land.publish(emp_msg);
    }
    
    void Controller::reset(){
        pub_empty_reset.publish(emp_msg);
    }
    
    void Controller::init(){
        while (msg_in_global.state != 1) {
            Controller::reset();
        }
    }
        
    
    double Controller::getPosX(int pixErrorX){
        double alphaX = ((msg_in_global.rotY*3.14)/180); // SKAL HENTES FRA QUADCOPTEREN
        double betaX = -atan(tan(GAMMA_X/2)*(pixErrorX)/PIXEL_DIST_X);
        double height = 1;//;msg_in_global.altd/1000; //HØJDEMÅLING FRA ULTRALYD
        return height * tan(alphaX+betaX);
    }
    double Controller::getPosY(int pixErrorY){
        double alphaY = ((msg_in_global.rotX*3.14)/180); // SKAL HENTES FRA QUADCOPTEREN
        double betaY = -atan(tan(GAMMA_Y/2)*(pixErrorY)/PIXEL_DIST_Y);
        double height = 1;//msg_in_global.altd/1000; //HØJDEMÅLING FRA ULTRALYD
        return height * tan(alphaY+betaY);
    }

    void Controller::auto_hover(){
    	pub_twist.publish(twist_msg_hover);
    }
	void Controller::update_state(Point2f center)
	{
		measured[X] = getPosX(center.y-PIXEL_DIST_X);
		measured[Y] = getPosY(center.x-PIXEL_DIST_Y);
	}

	void Controller::control(double dt)
	 {
	 	for(int i = 0; i < 4; i++)
	 	{
      		error[i] = target[i] - measured[i];
      		integral[i] = integral[i] + error[i] * dt;
      		derivative[i] = (error[i]-previous_error[i])/dt;
      		output[i] = Kp[i]*error[i] + Ki[i] * integral[i] + Kd[i] * derivative[i];
      		previous_error[i] = error[i];
    	}
    	if (output[X] > 0.1)
    	{
    		output[X] = 0.1;
    	}
    	if (output[X] < -0.1)
    	{
    		output[X] = -0.1;
    	}
    	if (output[Y] > 0.1)
    	{
    		output[Y] = 0.1;
    	}
    	if (output[Y] < -0.1)
    	{
    		output[Y] = -0.1;
    	}

    	//twist_msg.angular.z= output[ROT];
    	twist_msg.linear.x = output[X];
    	twist_msg.linear.y = output[Y];
    	//twist_msg.linear.z = output[Z];


    	pub_twist.publish(twist_msg);

    	ROS_INFO("Measured pos = (%g,%g)",measured[X],measured[Y]);
  		ROS_INFO("Output x: %g", output[X]);
  		ROS_INFO("Output y: %g", output[Y]);
	 }
}