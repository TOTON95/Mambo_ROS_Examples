/* \file mambo_guardian_angels.cpp
 * \author Alexis Guijarro
 * \license This project is released under the GNU Public License Version 3 (GPLv3)
 * \brief This is an experiment designed to run 3 Mambo Parrot Drone around an selected object tracked by mocap
 */

// Include the libraries
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
#include <string>
#include <math.h>

// Constants
#define PI 3.141592653589793238462

//Number of drones
#define N_DRONES 3                                 

double hdg;					//Heading of UAS							           
int btn_emergency;				//Physical emergency's button
bool on_live = false;				//The option to modify waypoints from topics    
std::vector<std::string> drones_topics;		//Topics with the drones
std::string target_topic;			//Topic with the target
Mambo_ROS_Examples::Waypoints wps;		//Collection of destination waypoints for drones
Mambo_ROS_Examples::Waypoint target;		//Target position
double altitude = 1.10				//Altitude of formation (meters)

std::vector<ros::Subscriber> drones_sub;		//Vector of subscribers
ros::Subscriber target_sub;			//Target subscriber

double offset_circle = 0;			//Offset of angle
double radius_circle = 1.25;			//Radius of formation (meters)

struct v_object
{
    double posX,posY,posZ;                         //Position of UAS
    double error_x,error_Y,error_Z;                //Error of the position of UAS
    double orX,orY,orZ,orW;                        //Orientation of UAS
    double roll,pitch,yaw,yawRAD;                  //Roll,Pitch,Yaw(deg),Yaw(Rad)
    double cmdX,cmdY,cmdZ,cmdYaw;                  //Command values
    double rot_cmd_x,rot_cmd_y;                    //Position in rotated matrix
    double velX,velY,velZ,velYaw;                  //Velocities
    double abs_x,abs_y;                            //Absolute position in X and Y
    double angles_res,angle_arc;                   //Angle resultant, angle arc
};

// Vehicles vector
std::vector<v_object> vehicles;

// Calculate the formation for each one of the drones following the target
bool calculate_formation(double offset, double radius, Mambo_ROS_Examples::Waypoints &wps)
{
    if(radius <= 0)
        return false;

    double slice = 2 * (PI/N_DRONES); 

    // Waypoints calculation
    for(int i=0;i<N_DRONES;i++)
    {
        double angle = slice * i;
        wps.wps[i].wp.x = target.wp.x + radius * cos(2*(angle + offset)); 
        wps.wps[i].wp.y = target.wp.y + radius * sin(2*(angle + offset)); 
        wps.wps[i].wp.z = altitude;
        wps.wps[i].hdg = 0;
    }
    return true; 
}

// Get the navigation info by vehicle index
void getNAVinfo(const geometry_msgs::TransformStamped::ConstPtr& pos, int index)
{
    vehicles[index+1].posX = pos -> transform.translation.x;       //Position in X
    vehicles[index+1].posY = pos -> transform.translation.y;       //Position in Y
    vehicles[index+1].posZ = pos -> transform.translation.z;       //Position in Z
    vehicles[index+1].orX = pos->transform.rotation.x;             //Rotation in X
    vehicles[index+1].orY = pos->transform.rotation.y;             //Rotation in Y
    vehicles[index+1].orZ = pos->transform.rotation.z;             //Rotation in Z
    vehicles[index+1].orW = pos->transform.rotation.w;             //Rotation in W

    tf::Quaternion q(pos->transform.rotation.x,pos->transform.rotation.y,pos->transform.rotation.z,pos->transform.rotation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(vehicles[index+1].roll,vehicles[index+1].pitch,vehicles[index+1].yawRAD);      //Get the Roll, Pitch, Yaw (Radians)
    vehicles[index+1].yaw = vehicles[index+1].yawRAD*(180/PI);                              //Convert the Yaw (Radians) into Yaw (Degrees)
    hdg = vehicles[index+1].yaw;                                                            //Set the hdg of the drone
    vehicles[index+1].abs_x = vehicles[index+1].posX;                                       //Set the absolute position of the drone in X
    vehicles[index+1].abs_y = vehicles[index+1].posY;                                       //Set the absolute position of the drone in Y

    if(index == -1)
    {
        target.wp.x = vehicles[index+1].posX;
        target.wp.y = vehicles[index+1].posY;
        target.wp.z = vehicles[index+1].posZ;
        target.hdg = vehicles[index+1].yaw;
    }
}

// Acquire the spatial data from each one of the agents
void getDronesInfo(ros::NodeHandle &n)
{
    // Get drones info
    for(int i=0;i<N_DRONES;i++)
        drones_sub[i] = n.subscribe<geometry_msgs::TransformStamped>(drones_topics[i], 100, boost::bind(getNAVinfo, _1, i));

    // Get target info
    target_sub = n.subscribe<geometry_msgs::TransformStamped>(target_topic, 100, boost::bind(getNAVinfo, _1, -1));
}

// Load parameters
bool loadParam(ros::NodeHandle &n)
{
    // Resize vectors to allocate the vehicles
    drones_topics.resize(N_DRONES);
    vehicles.resize(N_DRONES + 1);
    wps.wps.resize(N_DRONES);
    drones_sub.resize(N_DRONES);

    if(!n.getParam("uas_1",drones_topics[0]))
    {
        ROS_ERROR("\nUAS_1 tag not present\n");
        return false;
    }
    if(!n.getParam("uas_2",drones_topics[1]))
    {
        ROS_ERROR("\nUAS_2 tag not present\n");
        return false;
    }
    if(!n.getParam("uas_3",drones_topics[2]))
    {
        ROS_ERROR("\nUAS_3 tag not present\n");
        return false;
    }
    if(!n.getParam("uas_target",target_topic))
    {
        ROS_ERROR("\nUAS_TARGET tag not present\n");
        return false;
    }

    return true;
}

// Callback for joystick signals
void getJoy(const sensor_msgs::Joy::ConstPtr& msg)
{
    offset_circle += (double) msg -> buttons[6]*0.01;
    offset_circle -= (double) msg -> buttons[7]*0.01;
    ROS_INFO("\n%lf\n",offset_circle);
}

// Main function
int main (int argc, char* argv[])
{
    ros::init(argc,argv,"mambo_angels", ros::init_options::AnonymousName);
    ros::NodeHandle n;

    // Load parametes & prepare elements
    if(loadParam(n))
        getDronesInfo(n);
    else
    {
        ROS_ERROR("\nParameters not available\n");
        return -1;
    }

    // Create publisher
    ros::Publisher cmd_1,cmd_2,cmd_3;
    cmd_1 = n.advertise<Mambo_ROS_Examples::Waypoint>("wp_uas_1",100);
    cmd_2 = n.advertise<Mambo_ROS_Examples::Waypoint>("wp_uas_2",100);
    cmd_3 = n.advertise<Mambo_ROS_Examples::Waypoint>("wp_uas_3",100);

    // Get Joystick info
    ros::Subscriber joy_sub = n.subscribe("/joy",100,getJoy);

    ros::Rate r(100);
    while(n.ok())
    {
        ros::spinOnce();
        calculate_formation(offset_circle,radius_circle,wps);
        cmd_1.publish(wps.wps[0]);
        cmd_2.publish(wps.wps[1]);
        cmd_3.publish(wps.wps[2]);
        ROS_INFO("\nUAS_1 -> x:%lf y:%lf, UAS_2 -> x:%lf y:%lf, UAS_3 -> x:%lf y:%lf\nOffset_circle: %lf", wps.wps[0].wp.x,wps.wps[0].wp.y,wps.wps[1].wp.x,wps.wps[1].wp.y,wps.wps[2].wp.x,wps.wps[2].wp.y, offset_circle);
        r.sleep();
    }
    return 0;
}
