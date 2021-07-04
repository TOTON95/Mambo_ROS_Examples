# Mambo_ROS_Examples 

![Mambo ROS Examples Artwork](https://toton95.github.io/assets/img/posts/Mambo_ROS_Examples_2.jpg)

This is a collection based on my previous repository ([Bebop_ROS_Examples](https://github.com/TOTON95/Bebop_ROS_Examples)), but for this time, this collection is targeted to Mambo Parrot Drone using [ros_pyparrot](https://github.com/TOTON95/ros_pyparrot). This collection contains experiments with and without a Motion Capture System (Vicon), with one or more drones. Additionally, you can test our robust control strategy with perturbation estimation by cloning [ros_rhinf](https://github.com/TOTON95/ros_rhinf) (required for rhinf experiments).

## Credit

If you find this code useful for your project, please consider to cite us as:

*Bibtex:* 

```BibTeX
@ARTICLE{Rubio-Scola et al.:2020,
  author={I. R. {Scola} and G. A. G. {Reyes} and L. R. G. {Carrillo} and J. P. {Hespanha} and L. {Burlion}},
  journal={IEEE Transactions on Control Systems Technology}, 
  title={A Robust Control Strategy With Perturbation Estimation for the Parrot Mambo Platform}, 
  year={2020},
  pages={1-16},
  doi={10.1109/TCST.2020.3020783}}
```

## Quick-start:

1. Create and move to a new catkin workspace:

	`mkdir ~/mambo_ws && cd ~/mambo_ws`
   
2. Create and move to a new `src` folder:
	
	`mkdir src && cd src`

3. Check the requirements of  `ros_pyparrot` driver and clone it:

	`git clone https://github.com/TOTON95/ros_pyparrot.git`
	
4. Clone this repository:
	
	`git clone https://github.com/TOTON95/Mambo_ROS_Examples.git`
	
5. Install `pid_control` package for ROS:

   `sudo apt-get install ros-$ROS_DISTRO-pid`

6. **(For rhinf experiments)** Clone `ros_rhinf` repository:

   `git clone  https://github.com/TOTON95/ros_rhinf.git`

7. (**For Mocap experiments (Vicon)**) Clone [vicon_bridge](https://github.com/ethz-asl/vicon_bridge) and follow its instructions to configurate your environment:

   `git clone https://github.com/ethz-asl/vicon_bridge.git`
   
8. Move back to the `mambo_ws` directory:

   `cd ~/mambo_ws`

9. Build the experiments:

   `catkin_make`

10. Get ready the experiments:

    `source devel/setup.bash`

**You're done!  Please see the experiments below to choose one that fits your needs!**

## Disclaimer

The usage of this code comes without any warranties and responsibility falls completely up to who downloads it and executes the included experiments. 

These experiments can be executed by using this command:

`rosrun Mambo_ROS_Examples desired_node`

For example:

`rosrun Mambo_ROS_Examples mambo_simple_test`

**Warning:** Some nodes depend on others, please check the following section to make sure of fulfilling the requirements. An [example_node_configuration](https://github.com/TOTON95/Mambo_ROS_Examples/blob/development/Extras/example_node_configuration.png) is available as a visual aid.

## Included Experiments 

- **mambo_simple_test**: This is an open loop test (no Mocap required), which includes three different behaviors, more instructions on comments.
- **mambo_waypoints**: This is the core navigation system by waypoints for the Mambo drone, requires of the Mocap system as feedback. No waypoints provided at the start, will allow the system to accept waypoints on the fly. It requires a control node, such as `pid` or `rhinf`.
- **mambo_waypoint_circle**: This is an experiment designed to move a Mambo Parrot in a circle, it requires the `mambo_waypoints` node to work.
- **mambo_waypoint_time**: This is an experiment designed to run a Mambo Parrot drone through a series of waypoints based on time lapses, it requires the `mambo_waypoints` node to work.
- **mambo_guardian_angels**: This is an experiment designed to run 3 Mambo Parrot Drone around an selected object tracked by Mocap, it requires the `mambo_waypoints` node to work.

## Launch files 

The following launch files are a quick way to start an experiment, with pre-configurated variables, like the active controllers and their gains, Mocap names of the vehicles to track, among others. Please considerate to modify them to meet your requirements before start any experiment.

They can be launched as `roslaunch Mambo_ROS_Examples {desired_test}.launch`, for example:

`roslaunch Mambo_ROS_Examples test_square_PD_XY_time.launch`

Here is the list of the launch files that this collection contains, look into the launch files to watch which nodes are invoked:

- test_1.launch
- test_guardians.launch
- test_square.launch
- test_square_PD_XY_time.launch
- test_square_PD_XY_time_no_record.launch
- test_square_rhinf_XY.launch
- test_square_rhinf_XY_time.launch
- test_zero.launch
- test_zero_rhinf.launch
- test_zero_rhinf_XY.launch

### Extra: ROS Bags with Mocap Data of these experiments are available upon request



