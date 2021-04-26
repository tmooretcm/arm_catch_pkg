#ifndef ARM_SUBSCRIBER
#define ARM_SUBSCRIBER

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>


class ArmSubscriber{
public:
	ArmSubscriber(ros::NodeHandle &n);
	void poseCallback(const geometry_msgs::Pose &msg);

protected:
	const std::string _PLANNING_GROUP;
	moveit::planning_interface::MoveGroupInterface _move_group;
	moveit::planning_interface::PlanningSceneInterface _planning_scene_interface;
	const robot_state::JointModelGroup* _joint_model_group;
	moveit_visual_tools::MoveItVisualTools _visual_tools;
	ros::Subscriber _pose_sub;


};



#endif