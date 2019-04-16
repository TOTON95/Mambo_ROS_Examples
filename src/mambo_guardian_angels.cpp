#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_datatypes.h>
#include <Mambo_ROS_Examples/Waypoints.h>
#include <Mambo_ROS_Examples/Waypoint.h>
#include <ros/ros.h>
#include <stdlib.h>
#include <time.h>
#include <iostream>
#include <vector>

#define PI 3.141592653589793238462

double hdg;								//Heading of UAS
int btn_emergency;							//Physical emergency's button
bool on_live = false; 							//The option to modify waypoints from topics
geometry_msgs::Twist vel_mambo;						//Velocity data for each UAS
double ce_hdg, ce_pos_X, ce_pos_Y, ce_alt;				//Control efforts for PD controller
std_msgs::Float64 st_pos_x, st_pos_Y, st_pos_Z, st_hdg;			//States variables
std_msgs::Float64 stp_pos_x, stp_pos_Y, stp_pos_Z, stp_hdg;		//Setpoints variables


int main (int argc, char* args[])
{
	return 0;
}
