//ROS Headers
#include <ros/ros.h>

//Mambo_ROS_Examples headers
#include <Mambo_ROS_Examples/Waypoints.h>
#include <Mambo_ROS_Examples/Waypoint.h>

//Std Headers
#include <stdlib.h>
#include <time.h>
#include <iostream>
#include <vector>

//Variables
Mambo_ROS_Examples::Waypoints waypoints;						//Vector with waypoints
Mambo_ROS_Examples::Waypoint t_wps;							//Current waypoint
unsigned int c_wps = 0;									//Current waypoint index
std::vector<double> x,y,z;								//Parameter vectors
std::vector<double> headings;								//Heading vectors
std::vector<double> tpw;								//Time per waypoint
ros::Time _time, _actual_time, _last_time;						//ROS time variables 

//ROS Comm
ros::Publisher wp_pub;									//Waypoint publisher

//Get waypoint and publish it to the driver
bool sendWaypoint()
{
	//Check size
	if(c_wps > tpw.size())
		return false;

	ROS_INFO("CURRENT WAYPOINT -> %d",c_wps);

	//Get actual time + requested time
	_time =  ros::Time::now() + ros::Duration(tpw[c_wps]);

	//Get actual time and check that it did not surpassed requested time
	while(ros::Time::now().toSec() < _time.toSec())
	{
		t_wps.wp.x = x[c_wps];
		t_wps.wp.y = y[c_wps];
		t_wps.wp.z = z[c_wps];
		t_wps.hdg = headings[c_wps];
		wp_pub.publish(t_wps);
	}

	//Next waypoint
	c_wps++;
	return true;
}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"mambo_wp_time", ros::init_options::AnonymousName);		//InitROS node
	ros::NodeHandle n;								//Creates the node handle

	//Getting parameters
	bool load_param = true;

	if(!n.getParam("t_X",x))
        {
                ROS_ERROR("\nFailed to load the X parameters\n");
                load_param = false;
        }
        if(!n.getParam("t_Y",y))
        {
                ROS_ERROR("\nFailed to load the Y parameters\n");
                load_param = false;
        }
        if(!n.getParam("t_Z",z))
        {
                ROS_ERROR("\nFailed to load the Z parameters\n");
                load_param = false;
        }
        if(!n.getParam("t_Hdg",headings))
        {
                ROS_ERROR("\nFailed to load the Headings parameters\n");
                load_param = false;
	}
        if(!n.getParam("tpw",tpw))
        {
                ROS_ERROR("\nFailed to load the tpw parameters\n");
                load_param = false;
	}
	if(!load_param)
	{
		ROS_ERROR("\nParameters not correct, aborting...");
		return -1;
	}

	//Advertising topics
	wp_pub = n.advertise<Mambo_ROS_Examples::Waypoint>("wp",1);
	ros::Rate r(100);

	//Get data 
	ros::spinOnce();

	//Main loop
	while(n.ok() && sendWaypoint()) 
	{
		ros::spinOnce();
		r.sleep();
	}
}
