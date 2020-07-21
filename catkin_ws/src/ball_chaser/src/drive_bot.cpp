#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

//Publish motor commands
ros::Publisher motor_command_publisher;

//Callback function to publish linear x and angular z and return feedback mssg with wheel velocities

bool handle_drive_request(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& res){
	ROS_INFO("DriveToTargetRequest received - linear_x: %1.2f, angular_x: %1.2f",(float)req.linear_x,(float)req.angular_z);
	
	//Publish vel and angles to drive robot
	geometry_msgs::Twist motor_command;
	
	motor_command.linear.x=req.linear_x;
	motor_command.angular.z=req.angular_z;

	motor_command_publisher.publish(motor_command);

	res.msg_feedback = "Velocities are set at - linear_x: " + std::to_string(motor_command.linear.x) + ", angular_z: " + std::to_string(motor_command.angular.z);

	ROS_INFO_STREAM(res.msg_feedback);

	return true;
}

int main(int argc, char** argv){
	
	//Initialize ros node
	ros::init(argc,argv, "drive_bot");
	
	//Create ROS NodeHandle object
	ros::NodeHandle n;

	//Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
	motor_command_publisher=n.advertise<geometry_msgs::Twist>("/cmd_vel",10);

	//Define a drive /ball_chaser/command_robot service with the handle_drive_request callback function
	ros::ServiceServer service =n.advertiseService("ball_chaser/command_robot",handle_drive_request);
	ROS_INFO("Ready to send robot velocities");

	//Handle ROS communication events
	ros::spin();

	return 0;
}
