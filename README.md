# RoboND-Go-Chase-It
Project 2 of Udacity Robotics Software Engineer Nanodegree Program

![Gif of the world](Images/world.gif)
  
## Overview  
In this project, I create two ROS packages inside the catkin_ws/src: the drive_bot and the ball_chaser. Here are the steps to design the robot, house it inside your world, and program it to chase white-colored balls

1. drive_bot:

Create a my_robot ROS package to hold your robot, the white ball, and the world.
Design a differential drive robot with the Unified Robot Description Format. Add two sensors to your robot: a lidar and a camera. Add Gazebo plugins for your robot’s differential drive, lidar, and camera. The robot you design should be significantly different from the one presented in the project lesson. Implement significant changes such as adjusting the color, wheel radius, and chassis dimensions. Or completely redesign the robot model! After all you want to impress your future employers :-D
House your robot inside the world you built in the Build My World project.
Add a white-colored ball to your Gazebo world and save a new copy of this world.
The world.launch file should launch your world with the white-colored ball and your robot.

2. ball_chaser:

Create a ball_chaser ROS package to hold your C++ nodes.
Write a drive_botC++ node that will provide a ball_chaser/command_robot service to drive the robot by controlling its linear x and angular z velocities. The service should publish to the wheel joints and return back the requested velocities.
Write a process_image C++ node that reads your robot’s camera image, analyzes it to determine the presence and position of a white ball. If a white ball exists in the image, your node should request a service via a client to drive the robot towards it.
The ball_chaser.launch should run both the drive_bot and the process_image nodes.

## Prerequisites/Dependencies  
* Gazebo >= 7.0  
* ROS Kinetic  
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
## Setup Instructions (abbreviated)  
1. Meet the `Prerequisites/Dependencies`  
2. Open Ubuntu Bash and clone the project repository  
3. On the command line execute  
```bash
sudo apt-get update && sudo apt-get upgrade -y
```
4. Build and run your code.  
## Project Description  
Directory Structure  
```
Go Chase It                          # Go Chase It Project
    ├── my_robot                       # my_robot package                   
    │   ├── launch                     # launch folder for launch files   
    │   │   ├── robot_description.launch
    │   │   ├── world.launch
    │   ├── meshes                     # meshes folder for sensors
    │   │   ├── hokuyo.dae
    │   ├── urdf                       # urdf folder for xarco files
    │   │   ├── my_robot.gazebo
    │   │   ├── my_robot.xacro
    │   ├── world                      # world folder for world files
    │   │   ├── my_world.world
    │   │   ├── rrl_map.world
    │   ├── CMakeLists.txt             # compiler instructions
    │   ├── package.xml                # package info
    ├── ball_chaser                    # ball_chaser package                   
    │   ├── launch                     # launch folder for launch files   
    │   │   ├── ball_chaser.launch
    │   ├── src                        # source folder for C++ scripts
    │   │   ├── drive_bot.cpp
    │   │   ├── process_images.cpp
    │   ├── srv                        # service folder for ROS services
    │   │   ├── DriveToTarget.srv
    │   ├── CMakeLists.txt             # compiler instructions
    │   ├── package.xml                # package info                  
    └──                          
```
- [robot_description.launch](my_robot/launch/robot_description.launch): Launch file for robot
- [world.launch](my_robot/launch/world.launch): Launch file for world
- [hokuyo.dae](my_robot/meshes/hokayu.dae): Mesh file for robot
- [my_robot.gazebo](my_robot/urdf/my_robot.gazebo): Gazebo world plugin C++ script.
- [my_robot.xacro](my_robot/urdf/my_robot.xacro): Xacro file for robot model.
- [my_world.world](my_robot/world/my_world.world): Gazebo world file.  
- [rrl_map.world](my_robot/world/rrl_map.world): Gazebo world file of UMD Robotics Realization Lab

- [ball_chaser.launch](ball_chaser/launch/ball_chaser.launch): Launch file for ball chaser application
- [drive_bot.cpp](ball_chaser/src/drive_bot.cpp): C++ script to drive the robot
- [process_images.cpp](ball_chaser/src/process_images.cpp): C++ script to process camera images
- [DriveToTarget.srv](ball_chaser/srb/DriveToTarget.srv): Service file to get robot velocities

- [CMakeLists.txt](CMakeLists.txt): File to link the C++ code to libraries.  

## Run the project  
* Clone this repository
* At the top level of the project repository, create a build directory:  
```bash
mkdir build && cd build
```
* In `/build` directory, compile your code with  
```bash
cmake .. && make
```
* Launch the robot inside your world
```bash
source devel/setup.bash
roslaunch my_robot world.launch
```
* Run drive_bot and process_image
```bash
source devel/setup.bash
roslaunch ball_chaser ball_chaser.launch
```
* Visualize
```bash
source devel/setup.bash
rosrun rqt_image_view rqt_image_view 
```
* Move the ball and track it with the robot

## Tips  
1. It's recommended to update and upgrade your environment before running the code.  
```bash
sudo apt-get update && sudo apt-get upgrade -y
```