#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

//Define a global client that can request services

ros::ServiceClient client;

void drive_robot(float lin_x, float ang_z){
	ROS_INFO_STREAM("Send request to move the robot");

	ball_chaser::DriveToTarget srv;
	srv.request.linear_x=lin_x;
	srv.request.angular_z=ang_z;
	
	//Call command_robot service and pass requested velocities
	if(!client.call(srv)){
		ROS_ERROR("Failed to call service drive_robot");
	}
}

void process_image_callback(const sensor_msgs::Image img)
{
	int white_pixel=255, white_w=-1;

	//Loop through each pixel in the image and check if there's a bright white one
	for (int i=0;i<img.height*img.step;i++){
		if ((img.data[i]==255) && (img.data[i+1] == 255) && (img.data[i+2] == 255)){
			white_w=i%img.step;
			break;
		}
	}

	// Identify if this pixel falls in the left, mid, or right side of the image
	float lin_x=0.0,ang_z=0.0;
	if (white_w<=img.step*0.33 && white_w>=0){ //left side
		ang_z=0.5;	
	}
	else if(white_w>img.step*0.66 && white_w<=img.step){ //right side
		ang_z=-0.5;	
	}
	else if (white_w!=-1){ //centre 
		lin_x=0.5;
	}

	// Call the drive_bot function based on white_w position and pass velocities
	drive_robot(lin_x,ang_z);

	// Request a stop when there's no white ball seen by the camera

	if (white_w==-1){
		ROS_INFO_STREAM("Target not found: Stopping robot...");
	}
	else{
		ROS_INFO_STREAM("Target Detected: Driving to target");
	}
}

int main(int argc,char** argv){
	//Initialize process image node and create node handle
	
	ros::init(argc, argv, "process image");
	ros::NodeHandle n;

	//Define a client service capable of requesting services from command_robot
	client=n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

	//Subscribe to camera/rgb/image_raw topic to read the image data inside the process_image_callback function 
	ros::Subscriber sub1=n.subscribe("/camera/rgb/image_raw",10,process_image_callback);

	//Handle ROS communication events
	ros::spin();

	return 0;

}
