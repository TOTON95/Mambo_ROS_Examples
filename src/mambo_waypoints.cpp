//Coded by Alexis Guijarro

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
#include <Mambo_ROS_Examples/ArcDrone.h>
#include <ros/ros.h>
#include <stdlib.h>
#include <time.h>
#include <iostream>
#include <vector>

#define PI 3.141592653589793238462

double hdg;                                                             //Heading of UAS
int btn_emergency;                                                      //Physical emergency's button
bool on_live = false;                                                   //The option to modify waypoints from topics
geometry_msgs::Twist cmd_vel_mambo;                                     //Velocity data for each UAS
double ce_hdg, ce_pos_X, ce_pos_Y, ce_alt;                              //Control efforts for PD controller
std_msgs::Float64 st_pos_X, st_pos_Y, st_alt, st_hdg;			//States variables
std_msgs::Float64 stp_pos_X, stp_pos_Y, stp_alt, stp_hdg;		//Setpoints variables
Mambo_ROS_Examples::Waypoints waypoints;				//Vector with waypoints
Mambo_ROS_Examples::Waypoint t_wps;					//Current waypoint
unsigned int c_wps = 0;							//Current waypoint index
double hdg_target; 							//Heading target
double goal_bound = 0.05;						//Boundaries of the goal
int wps;								//Number of waypoints
std::vector<double> x,y,z;						//Parameter vectors
std::vector<double> headings;						//Heading vectors
double time_goal;							//Time to stay at waypoint
ros::WallTime _time;							//Time of the system
ros::WallTime _actual_time;						//Actual Time
ros::WallTime _last_time; 						//Last time captured
double _time_data;							//Raw time data
double _time_secs;							//Time in seconds
double _time_wait;						

bool ready = false;							//Ready to get orders

struct v_object
{
	double posX,posY,posZ;                                          //Position of UAS
	double error_x,error_Y,error_Z;                                 //Error of the position of UAS
	double orX,orY,orZ,orW;                                         //Orientation of UAS
	double roll,pitch,yaw,yawRAD;                                   //Roll,Pitch,Yaw(deg),Yaw(Rad)
	double cmdX,cmdY,cmdZ,cmdYaw;                                   //Command values
	double rot_cmd_x,rot_cmd_y;                                     //Position in rotated matrix
	double velX,velY,velZ,velYaw;                                   //Velocities
	double abs_x,abs_y;                                             //Absolute position in X and Y
	double angles_res,angle_arc;                                    //Angle resultant, angle arc
}mambo;

void getJoyState(const sensor_msgs::Joy::ConstPtr& js)                  //Capture Joystick data
{
	btn_emergency = js->buttons[0];
	if(js->buttons[4]) c_wps++;
}

void getMamboPos(const geometry_msgs::TransformStamped::ConstPtr& pos)  //Capture VICON Data
{
	mambo.posX = pos->transform.translation.x;                     //Position in X
	mambo.posY = pos->transform.translation.y;                     //Position in Y
	mambo.posZ = pos->transform.translation.z;                     //Position in Z
	mambo.orX = pos->transform.rotation.x;                         //Rotation in X
	mambo.orY = pos->transform.rotation.y;                         //Rotation in Y
	mambo.orZ = pos->transform.rotation.z;                         //Rotation in Z
	mambo.orW = pos->transform.rotation.w;                         //Rotation in W

	tf::Quaternion q(pos->transform.rotation.x,pos->transform.rotation.y,pos->transform.rotation.z,pos->transform.rotation.w);
	tf::Matrix3x3 m(q);
	m.getRPY(mambo.roll,mambo.pitch,mambo.yawRAD);               	//Get the Roll, Pitch, Yaw (Radians)
	mambo.yaw = mambo.yawRAD*(180/PI);                            	//Convert the Yaw (Radians) into Yaw (Degrees)
	hdg = mambo.yaw;                                           	//Set the hdg of the drone
	mambo.abs_x = mambo.posX;                                      //Set the absolute position of the drone in X
	mambo.abs_y = mambo.posY; //Set the absolute position of the drone in Y
}

//Avoids problems with the hdg of the motion capture system

double GetAngleDifference(double from, double to)
{
	double difference = to - from;
	while (difference < -180) difference += 360;
	while (difference > 180) difference -= 360;
	return difference;
}

//Get Control Efforts

void getEffort_pos_X(const std_msgs::Float64::ConstPtr& msg)
{
	ce_pos_X = msg->data;
}
void getEffort_pos_Y(const std_msgs::Float64::ConstPtr& msg)
{
	ce_pos_Y = msg->data;
}
void getEffort_alt(const std_msgs::Float64::ConstPtr& msg)
{
	ce_alt = msg->data;
}
void getEffort_hdg(const std_msgs::Float64::ConstPtr& msg)
{
	ce_hdg = -msg->data;
}

//Set waypoints from parameters
bool setWayPoints()
{
	if(x.size() != wps || y.size() != wps || z.size() != wps || headings.size() != wps)
	{
		ROS_ERROR("\nNumber of waypoints does not match with number of positions!\n");
		return true;
	}



	for(int k=0;k<wps;k++)
	{
		Mambo_ROS_Examples::Waypoint z0;
		z0.wp.x = x[k];
		z0.wp.y = y[k];
		z0.wp.z = z[k];
		z0.hdg = headings[k];

		waypoints.wps.push_back(z0);
	}

	return false;
}

void getWP(const Mambo_ROS_Examples::Waypoint::ConstPtr& msg)
{
	Mambo_ROS_Examples::Waypoint ct;
	ct.wp.x = msg->wp.x;
	ct.wp.y = msg->wp.y;
	ct.wp.z = msg->wp.z;
	ct.hdg = msg -> hdg;

	t_wps = ct;
}

void getReady(const std_msgs::String::ConstPtr& msg)
{
	if(msg->data == "ok")
		ready = true;
}

//Get Waypoints from existent topic
void getWaypoints(const Mambo_ROS_Examples::Waypoints::ConstPtr& msg)
{
	waypoints.wps = msg -> wps;
}

//Procedure to reach every waypoint
bool getWayPointInfo()
{
	if(c_wps  > (waypoints.wps.size()-1))
	{
		ROS_INFO("\n\n\nReached end of the routine\n\n\n");
		return false;
	}

	t_wps = waypoints.wps[c_wps];						//Assign stored waypoint to current waypoint

	bool x_no_top_reached = mambo.posX < (waypoints.wps[c_wps].wp.x + goal_bound);
	bool x_no_bottom_reached = mambo.posX > (waypoints.wps[c_wps].wp.x - goal_bound);
	bool y_no_top_reached = mambo.posY < (waypoints.wps[c_wps].wp.y + goal_bound);
	bool y_no_bottom_reached = mambo.posY > (waypoints.wps[c_wps].wp.y - goal_bound);
	bool z_no_top_reached = mambo.posZ < (waypoints.wps[c_wps].wp.z + goal_bound);
	bool z_no_bottom_reached = mambo.posZ > (waypoints.wps[c_wps].wp.z - goal_bound);

	if(x_no_top_reached && x_no_bottom_reached && y_no_top_reached && y_no_bottom_reached && z_no_top_reached && z_no_bottom_reached)
	{
		_time = ros::WallTime::now();                                   //Get the current time
		_actual_time = _time;                                           //Save the current time
		_time_secs = _actual_time.toSec() - _last_time.toSec();         //Get the time of the order in seconds
		_actual_time = _last_time;					//Set current time to be the latest
		_time_data += _time_secs;                                       //Sum the seconds to count every second inside the zone
		if(_time_data > time_goal)                                      //If the drone stays more than x seconds in the zone
		{
			_time_data = 0;
			ROS_INFO("\n<--Waypoint %d reached-->\n",c_wps);
			c_wps++;
		}
	}
	return true;
}

//Main function
int main(int argc, char** argv)
{
	ros::init(argc,argv,"mambo_waypoints", ros::init_options::AnonymousName);	//InitROS node
	ros::NodeHandle n;                                              //Creates the node handle
	ros::Subscriber joy_sub,mambo_sub;                              //Joystick and Vicon subs
	ros::Subscriber pid_pos_X,pid_pos_Y,pid_hdg,pid_alt;            //PID controllers subs
	ros::Subscriber wps_sub;					//Waypoints Sub
	ros::Subscriber wp_sub;						//Waypoint Sub
	ros::Subscriber ready_sub;					//Ready Sub
	ros::Publisher state_pos_X,state_pos_Y,state_hdg,state_alt;     //Current state of the drone
	ros::Publisher sp_pos_X,sp_pos_Y,sp_hdg,sp_alt;                 //Set the goal of the drone
	ros::Publisher sp_real_hdg;					//Real hdg setpoint
	ros::Publisher tko,land;                                        //Take-off and landing publisher
	ros::Publisher cmd_vel;                                         //Velocity command of the drone
	ros::Publisher record;						//Signal to record
	ros::Publisher hdg_pub;						//COnverted heading

	std::string vehicle_name;					//Vehicle name at VICON

	//Getting parameters
	bool load_param = true;
	if(!n.getParam("waypoints",wps))
	{
		ROS_ERROR("\nFailed to load the waypoints parameters\n");
		load_param = false;
	}
	if(!n.getParam("X",x))
	{
		ROS_ERROR("\nFailed to load the X parameters\n");
		load_param = false;
	}
	if(!n.getParam("Y",y))
	{
		ROS_ERROR("\nFailed to load the Y parameters\n");
		load_param = false;
	}
	if(!n.getParam("Z",z))
	{
		ROS_ERROR("\nFailed to load the Z parameters\n");
		load_param = false;
	}
	if(!n.getParam("Hdg",headings))
	{
		ROS_ERROR("\nFailed to load the Headings parameters\n");
		load_param = false;
	}
	n.param("Time_goal",time_goal,0.05);
	if(!n.getParam("mocap_name", vehicle_name))
	{
		ROS_ERROR("\nFailed to load Vehicle Name\n");
		vehicle_name = "Mambo_5";
	}

	//Load the parameters
	if(load_param)
	{
		if(setWayPoints())
			return -1;
	}
	else
	{
		ROS_ERROR("\nStarting at the center of the world, ready to receive waypoints!\n");
		t_wps.wp.x = 0;
		t_wps.wp.y = 0;
		t_wps.wp.z = 1.05;
		t_wps.hdg = 0;
		waypoints.wps.push_back(t_wps);
		on_live = true;
		if(on_live)
			ROS_WARN("\nLive mode activated!\n");
	}


	//Subscribers of the ROS node
	joy_sub = n.subscribe("/joy",1000,getJoyState);
	mambo_sub = n.subscribe("/vicon/"+vehicle_name+"/"+vehicle_name,1000,getMamboPos);
	pid_pos_X = n.subscribe("control_effort_pos_X",1000,getEffort_pos_X);
	pid_pos_Y = n.subscribe("control_effort_pos_Y",1000,getEffort_pos_Y);
	pid_alt = n.subscribe("control_effort_alt",1000,getEffort_alt);
	pid_hdg = n.subscribe("control_effort_hdg",1000,getEffort_hdg);
	wps_sub = n.subscribe("waypoints",1000,getWaypoints);
	wp_sub = n.subscribe("wp",1000,getWP);
	ready_sub = n.subscribe("ready",1000,getReady);


	//Publishers of the ROS node
	tko = n.advertise<std_msgs::Empty>("take_off",1000);
	land = n.advertise<std_msgs::Empty>("land",1000);
	cmd_vel = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
	state_pos_X = n.advertise<std_msgs::Float64>("state_pos_X",1000);
	state_pos_Y = n.advertise<std_msgs::Float64>("state_pos_Y",1000);
	state_alt = n.advertise<std_msgs::Float64>("state_alt",1000);
	state_hdg = n.advertise<std_msgs::Float64>("state_hdg",1000);
	sp_pos_X = n.advertise<std_msgs::Float64>("setpoint_pos_X",1000);
	sp_pos_Y = n.advertise<std_msgs::Float64>("setpoint_pos_Y",1000);
	sp_alt = n.advertise<std_msgs::Float64>("setpoint_alt",1000);
	sp_hdg = n.advertise<std_msgs::Float64>("setpoint_hdg",1000);
	record = n.advertise<std_msgs::Empty>("record",1000);
	sp_real_hdg = n.advertise<std_msgs::Float64>("r_setpoint_hdg",1000);
	hdg_pub = n.advertise<std_msgs::Float64>("hdg",1000);

	std_msgs::Empty msg_tko,msg_land;                                //Msgs to take-off and land

	//Making sure to set every velocity to 0
	cmd_vel_mambo.linear.x = 0;
	cmd_vel_mambo.linear.y = 0;
	cmd_vel_mambo.linear.z = 0;
	cmd_vel_mambo.angular.x = 0;
	cmd_vel_mambo.angular.y = 0;
	cmd_vel_mambo.angular.z = 0;

	ros::spinOnce();                                                 //Refresh the topics

	ros::Duration(2).sleep();                                        //Time necessary to setup
	ros::Rate r(100);                                                //Set node at 100Hz


	printf("\n========== T A K E O F F ==========\n");
	tko.publish(msg_tko);                                            //Take-off
	ros::Duration(5).sleep();                                      //Wait until the drone elevates

	while(n.ok() && !btn_emergency)
	{
		if(ready)
		{
			printf("\n=====START=====\n");
			break;
		}
		else
			printf("\n=====STAND-BY=====\n");
			
		ros::spinOnce();
		r.sleep();
				
	}


	record.publish(msg_tko);					 //Record here

	//Stops the node once that <Ctrl + C > is pressed
	while(n.ok())
	{
		//Lands the drone if the joystick's button is pressed
		if(btn_emergency)
		{
			ros::Duration(0.525).sleep();
			printf("\n========== L A N D [ J S ]==========\n");
			land.publish(msg_land);                           //Land the drone
			break;
		}

		if(!getWayPointInfo() && !on_live)			  //Get Waypoint information
		{
			ros::Duration(0.525).sleep();
			printf("\n========== L A N D [ E N D ]==========\n");
			//			land.publish(msg_land);                           //Land the drone if end is reached
			break;
		}

		//Wrapping the angle
		double diff = GetAngleDifference(hdg,t_wps.hdg);

		//Send real heading to topic
		std_msgs::Float64 r_hdg;
		r_hdg.data = t_wps.hdg;
		sp_real_hdg.publish(r_hdg);

		std_msgs::Float64 f_hdg;
		f_hdg.data = constrainAngle(hdg);
		hdg_pub.publish(f_hdg);

		//Updating Heading PID Controller
		st_hdg.data=diff;
		stp_hdg.data=0;
		ros::spinOnce();

		state_hdg.publish(st_hdg);
		sp_hdg.publish(stp_hdg);
		//ros::Duration(0.0001).sleep();

		ros::spinOnce();

		//Set hdg velocity
		cmd_vel_mambo.angular.z = ce_hdg;

		//Updating Position PID Controller
		ros::spinOnce();

		st_pos_X.data=mambo.posX;
		stp_pos_X.data=t_wps.wp.x;
		state_pos_X.publish(st_pos_X);
		sp_pos_X.publish(stp_pos_X);
		ros::spinOnce();
		mambo.cmdX = ce_pos_X;

		st_pos_Y.data=mambo.posY;
		stp_pos_Y.data=t_wps.wp.y;
		state_pos_Y.publish(st_pos_Y);
		sp_pos_Y.publish(stp_pos_Y);
		ros::spinOnce();
		mambo.cmdY = ce_pos_Y;

		//Calculating the rotation matrix of the drone
		mambo.rot_cmd_x = mambo.cmdX*cos(constrainAngle(hdg) * 0.0174533) + mambo.cmdY*sin(constrainAngle(hdg) * 0.0174533);
		mambo.rot_cmd_y = mambo.cmdX*sin(constrainAngle(hdg) * 0.0174533) - mambo.cmdY*cos(constrainAngle(hdg) * 0.0174533);

		//Set X and Y velocities
		cmd_vel_mambo.linear.x = mambo.rot_cmd_x;
		cmd_vel_mambo.linear.y = -mambo.rot_cmd_y;

		//Updating Altittude PID Controller
		st_alt.data = mambo.posZ;
		stp_alt.data = t_wps.wp.z;
		state_alt.publish(st_alt);
		sp_alt.publish(stp_alt);
		ros::spinOnce();

		//Set velocity in Z
		cmd_vel_mambo.linear.z = ce_alt;

		//Send the velocity command
		cmd_vel.publish(cmd_vel_mambo);

		ROS_INFO("X: %lf  Y: %lf HDG: %lf CMDX: %lf  CMDY: %lf CMDHDG: %lf WAYPOINT: %i",mambo.posX,mambo.posY, hdg, cmd_vel_mambo.linear.x,cmd_vel_mambo.linear.y,cmd_vel_mambo.angular.z,c_wps);

		ros::spinOnce();

		r.sleep();

	}

	ros::Duration(2).sleep(); 				//Wait 2 Seconds to finish
	ros::spinOnce();

	//Land the drone if it still flying
	while(mambo.posZ > 0.60)
	{
		ros::spinOnce();
		ros::Duration(0.525).sleep();
		printf("\n========== L A N D [ A L T ]==========\n");
		land.publish(msg_land);
	}

}
