#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Set the goal to pick-up zone
  goal.target_pose.pose.position.x = -0.5;
  goal.target_pose.pose.position.y = -4.0;
  goal.target_pose.pose.orientation.w = 1.0;
  ROS_INFO("Sending pickup goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("It's reached pickup goal!");
  else {
    ROS_INFO("Failed to move to pickup goal...");
    return 0;
  }

  // Wait for 5 seconds
  ros::Duration(5.0).sleep();
  
  // Set the goal to pick-up zone
  goal.target_pose.pose.position.x = -0.5;
  goal.target_pose.pose.position.y = -1.0;
  goal.target_pose.pose.orientation.w = 1.0;
  ROS_INFO("Sending drop off goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached drop off zone
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("It's reached drop off zone!");
  else {
    ROS_INFO("Failed to move to drop off zone...");
  }

  return 0;
}
