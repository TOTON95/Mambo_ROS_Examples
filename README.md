# Mambo_ROS_Examples 

This is a collection based on my previous repository ([Bebop_ROS_Examples](https://github.com/TOTON95/Bebop_ROS_Examples)), but for this time this collection is targeted to Mambo Parrot Drone using [ros_pyparrot](https://github.com/TOTON95/ros_pyparrot). This collection contains experiments with and without a Motion Capture System (Vicon), with one or more drones. Additionally, you can test our robust control strategy with perturbation estimation by cloning [ros_rhinf](https://github.com/TOTON95/ros_rhinf) (required for rhinf experiments).

## Credit

If you find this code useful for your project, please cite us as:

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
	
5. **(For rhinf experiments)** Clone `ros_rhinf` repository:

   `git clone  https://github.com/TOTON95/ros_rhinf.git`

6. (**For Mocap experiments (Vicon)**) Clone [vicon_bridge](https://github.com/ethz-asl/vicon_bridge) and follow its instructions to configurate your environment:

   `git clone https://github.com/ethz-asl/vicon_bridge.git`

## Disclaimer

The usage of this code comes without any warranties and responsibility falls completely up to who downloads it and executes the included experiments. 

## Included Experiments 

- **mambo_simple_test**: This is an open loop test (no Mocap required), which includes three different behaviours, more instructions on comments.
- **mambo_waypoints**: This is the core navigation system by waypoints for the Mambo drone, requires of the Mocap system as feedback. No waypoints provided at the start, will allow the system to accept waypoints on the fly. 
- **mambo_waypoint_circle**: This is an experiment designed to move a Mambo Parrot in a circle, it requires the `mambo_waypoints` node to work.
- **mambo_waypoint_time**: This is an experiment designed to run a Mambo Parrot drone through a series of waypoints based on time lapses, it requires the `mambo_waypoints` node to work.
- **mambo_guardian_angels**: This is an experiment designed to run 3 Mambo Parrot Drone around an selected object tracked by Mocap, it requires the `mambo_waypoints` node to work.



