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
#define KP_X 0.1
#define KP_Y 0.1
#define KP_ROT 0
#define KP_Z 0
#define X 0
#define Y 1
#define ROT 2
#define Z 3

//Husk at dobbelttjek disse værdier
#define GAMMA_X = 40 //grader
#define GAMMA_Y = 64
#define PIXEL_DIST_X = 180 //pixels
#define PIXEL_DIST_Y = 320



namespace controller
{
	Kp = {KP_X,KP_Y,KP_ROT,KP_Z};
	void Controller::nav_callback(const ardrone_autonomy::Navdata& msg_in)
	{
		//Take in navdata from ardrone
		msg_in_global = msg_in;
	}
	void Controller::takeoff(){
		pub_empty_takeoff.publish(emp_msg);
	}
	void Controller::begin(){
		double time_start=(double)ros::Time::now().toSec();
 		while (ros::ok() && ((double)ros::Time::now().toSec()< time_start+1)){
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
    
	void Controller::update_state()
	{
		//TODO: Add getposx og y og update measured
	}

	Controller::Controller()
	{
		nav_sub = node.subscribe("/ardrone/navdata", 1, &Controller::nav_callback,this);
		pub_twist = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1); /* Message queue length is just 1 */
		pub_empty_takeoff = node.advertise<std_msgs::Empty>("/ardrone/takeoff", 1); /* Message queue length is just 1 */
		pub_empty_land = node.advertise<std_msgs::Empty>("/ardrone/land", 1); /* Message queue length is just 1 */
		pub_empty_reset = node.advertise<std_msgs::Empty>("/ardrone/reset", 1); /* Message queue length is just 1 */
	}

	void Controller::control(int dt)
	 {
	 	for(int i = 0; i < 4; i++)
	 	{
      		error[i] = target[i] - measured[i];
      		integral[i] = integral[i] + error[i] * dt;
      		derivative[i] = (error[i]-previous_error[i])/dt;
      		output[i] = Kp[i]*error[i] + Ki[i] * integral[i] + Kd[i] * derivative[i];
      		previous_error[i] = error[i];
    	}
    	twist_msg.angular.z= output[ROT];
    	twist_msg.linear.x = output[X];
    	twist_msg.linear.y = output[Y];
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
}