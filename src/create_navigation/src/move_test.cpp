#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
 
int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_test");
 
  // subscribe move_base  
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base("move_base", true);
 
  // wait for connection, no longer than 5 sec 
  while(!move_base.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for move_base action server...");
  }
 
  ROS_INFO("Connected to move base server");

  // set goal
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
 
  goal.target_pose.pose.position.x = -0.300;
  goal.target_pose.pose.position.y = 1.000;

  goal.target_pose.pose.orientation.z = 0.000;
  goal.target_pose.pose.orientation.w = 0.764;
 
  ROS_INFO("Sending goal");
  move_base.sendGoal(goal);
 
  move_base.waitForResult();
 
  if(move_base.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Goal succeeded!");
  else
    ROS_INFO("Goal failed");
 
  return 0;

}
