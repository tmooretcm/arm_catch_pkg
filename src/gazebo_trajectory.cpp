#include "arm_test_pkg/gazebo_trajectory.h"


GazeboTrajectorySubscriber::GazeboTrajectorySubscriber(ros::NodeHandle &n){
	_sub = n.subscribe("/gazebo/model_states", 1000, &GazeboTrajectorySubscriber::trajectoryCB, this);
	_client = n.serviceClient<arm_catch_pkg::TrajectoryPredict>("trajectory_predict");
}

void GazeboTrajectorySubscriber::trajectoryCB(const gazebo_msgs::ModelStates &msg){
	arm_catch_pkg::TrajectoryPredict srv;

	for(int n = 0; n < msg.name.size(); n++){
		if(msg.name.at(n).compare("RoboCup SPL Ball") == 0){
			srv.request.start_pose = msg.pose.at(n);
			srv.request.start_twist = msg.twist.at(n);
			srv.request.time_s = 1;

			_client.call(srv);

			ROS_INFO("Trajectory Prediction Service Called");
			ROS_INFO("X: %f\nY: %f\nZ: %f", 
				srv.response.predicted_pose.position.x,
				srv.response.predicted_pose.position.y,
				srv.response.predicted_pose.position.z); 
			return;
		}
	}
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "gazebo_trajectory_client");
	ros::NodeHandle node_handle;

	GazeboTrajectorySubscriber gts(node_handle);
	
	ros::spin();

	return 0;
}