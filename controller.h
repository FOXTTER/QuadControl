#ifndef CONTROLLER_H

#define CONTROLLER_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Empty.h>
#include <ardrone_autonomy/Navdata.h>
#include <geometry_msgs/Twist.h>
#include <cv.h>
#include <highgui.h>

namespace controller{
	class Controller
	{
		public:
			Controller();
			void takeoff();
			void reset();
			void land();
			void control(int dt);
			void begin();
			void update_state();


		//private:
			ros::NodeHandle node;
  			ros::Publisher pub_empty_land;
			ros::Publisher pub_twist;
			ros::Publisher pub_empty_takeoff;
			ros::Publisher pub_empty_reset;
			ros::Subscriber nav_sub;
			geometry_msgs::Twist twist_msg;
			geometry_msgs::Twist twist_msg_hover;
			geometry_msgs::Twist twist_msg_pshover;
			std_msgs::Empty emp_msg;
			ardrone_autonomy::Navdata msg_in_global;
			double previous_error[4];
  			double error[4];
  			double output[4];
  			double derivative[4];
  			double integral[4];
  			double target[4];
			double Kp[4];
			double Ki[4];
			double Kd[4];
			void nav_callback(const ardrone_autonomy::Navdata& msg_in);
	};
}

#endif