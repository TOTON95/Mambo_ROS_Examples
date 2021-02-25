/* \file mambo_waypoint_circle.cpp
 * \author Alexis Guijarro
 * \license This project is released under the GNU Public License Version 3 (GPLv3)
 * \brief This is an experiment designed to move a Mambo Parrot in a circle, it requires 
 * the mambo_waypoints node to work
 */

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
#include <cmath>

//Variables
Mambo_ROS_Examples::Waypoints waypoints;				//Vector with waypoints
Mambo_ROS_Examples::Waypoint t_wps;						//Current waypoint
unsigned int c_wps = 0;									//Current waypoint angle
std::vector<double> x,y,z;								//Parameter vectors
std::vector<double> headings;							//Heading vectors
std::vector<double> tpw;								//Time per waypoint
ros::Time _time, _actual_time, _last_time;				//ROS time variables 

//ROS Comm
ros::Publisher wp_pub;									//Waypoint publisher

//Get waypoint and publish it to the driver
void sendWaypoint(unsigned int i);

//Get the coordinates from the circular path
std::vector<double> get_circle_coordinates(unsigned int t, double r=0.75);

int main(int argc, char* argv[])
{
    ros::init(argc,argv,"mambo_wp_time", ros::init_options::AnonymousName);		//InitROS node
    ros::NodeHandle n;								                            //Creates the node handle

    //Advertising topics
    wp_pub = n.advertise<Mambo_ROS_Examples::Waypoint>("wp",1);
    ros::Rate r(12);

    //Get data 
    ros::spinOnce();

    //Number of loops
    int n_loops = 5;

    //Current loop
    int c_loop = 0;

    //Main loop
    while(n.ok()) 
    {

        //Break once the number of loops are done
        if(c_loop < n_loops*360)
        {
            //Print INFO
            ROS_INFO("= = = = = LOOP[%u] = = = = =\n", c_loop/360);
            c_loop++;
        }

        else
            break;

        //Send waypoint
        sendWaypoint(c_wps);

        //Update and check values
        c_wps++;
        if(c_wps > 359) c_wps = 0;

        //Update ROS
        ros::spinOnce();
        r.sleep();
    }
}

// Send the current waypoint to nav system
void sendWaypoint(unsigned int i)
{
    //Calculate the waypoint
    std::vector<double> wp;

    wp = get_circle_coordinates(i);
    t_wps.wp.x = wp.at(0);
    t_wps.wp.y = wp.at(1);
    t_wps.wp.z = 1.10;
    t_wps.hdg = 0.0;
    wp_pub.publish(t_wps);

    ROS_INFO("X: %lf Y: %lf\n",t_wps.wp.x,t_wps.wp.y);
}

// Create the circle coordinates
std::vector<double> get_circle_coordinates(unsigned int t, double r)
{
    if(t > 359) t = 359;
    if(t < 0) t = 0;

    //Create array
    std::vector<double> coords;
    coords.resize(2);

    //Fill up coords
    coords.at(0) = r * cos(t * (3.1459 / 180.00));
    coords.at(1) = r * sin(t * (3.1459 / 180.00));

    //Return results
    return coords;
}
