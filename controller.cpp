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


namespace controller
{
	void Controller::nav_callback(const ardrone_autonomy::Navdata& msg_in)
	{
		//Take in state of ardrone
		msg_in_global = msg_in;
	}
	void Controller::takeoff
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
    
	Controller::Controller()
	{
		nav_sub = node.subscribe("/ardrone/navdata", 1, &Controller::nav_callback,this);
		pub_twist = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1); /* Message queue length is just 1 */
		pub_empty_takeoff = node.advertise<std_msgs::Empty>("/ardrone/takeoff", 1); /* Message queue length is just 1 */
		pub_empty_land = node.advertise<std_msgs::Empty>("/ardrone/land", 1); /* Message queue length is just 1 */
		pub_empty_reset = node.advertise<std_msgs::Empty>("/ardrone/reset", 1); /* Message queue length is just 1 */
	}

}