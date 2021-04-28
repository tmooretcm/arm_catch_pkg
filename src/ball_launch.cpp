#include <ros/ros.h>
#include "ros/service_client.h"
#include "gazebo_msgs/SetModelState.h"
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/ApplyBodyWrench.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ball_launch");
	ros::NodeHandle node_handle;


	ros::ServiceClient modelState_Client = node_handle.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

	geometry_msgs::Pose modelPose;
	modelPose.position.x = 0;
	modelPose.position.y = 6;
	modelPose.position.z = 0.5;
	modelPose.orientation.w = 1.0;
	modelPose.orientation.x = 0.0;
	modelPose.orientation.y = 0.0;
	modelPose.orientation.z = 0.0;

	gazebo_msgs::ModelState modelState;
	modelState.model_name = (std::string) "RoboCup SPL Ball";
	modelState.pose = modelPose;

	gazebo_msgs::SetModelState srv;
	srv.request.model_state = modelState;

	modelState_Client.call(srv);



	ros::ServiceClient abwClient = node_handle.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");

	geometry_msgs::Wrench wrench;
	wrench.force.x = 0.0;
	wrench.force.y = -30.0;
	wrench.force.z = 30.0;
	wrench.torque.x = 0.0;
	wrench.torque.y = 0.0;
	wrench.torque.z = 0.0;

	gazebo_msgs::ApplyBodyWrench abw;
	abw.request.body_name = (std::string) "RoboCup SPL Ball::ball";
	abw.request.wrench = wrench;
	abw.request.start_time = ros::Time::now();
	abw.request.duration = ros::Duration(0.001); // This was just empirically determined

	abwClient.call(abw);


	return 0;
}