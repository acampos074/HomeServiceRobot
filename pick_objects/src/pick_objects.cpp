#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/Bool.h"

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
// Global boolean marker state variable
ros::Publisher marker_state_pub;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  // TODO: edit the node name to pick_objects
  ros::init(argc, argv, "pick_objects");
  // Create a handle for this node
  ros::NodeHandle n;
  // Define a publisher to publish std_msgs::Bool messages on the state of the marker
  marker_state_pub = n.advertise<std_msgs::Bool>("/pick_objects/marker_state",10);

  // Create boolean variable state of the marker (pickup_zone = true: the package is currently in the pick up zone)
  std_msgs::Bool pickup_zone;

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  // TODO: edit the frame_id to map
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  // TODO: define desired pickup goal
  goal.target_pose.pose.position.x = 6.11;
  goal.target_pose.pose.position.y = -3.81;
  goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("The robot has reached its destination!");
    ros::Duration(5.0).sleep(); // sleep for five seconds
    // Update the pickup_zone topic to false, so that the robot can move to the drop off zone
    pickup_zone.data = false;
    marker_state_pub.publish(pickup_zone);

  }
  else
    ROS_INFO("The robot failed to move for some reason");

  // TODO: include an extra goal position and orientation for the robot to reached
  // This is the desired drop off goal
  goal.target_pose.pose.position.x = -5.85;
  goal.target_pose.pose.position.y = -5.55;
  goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The robot has reached the desired drop off zone!");
  else
    ROS_INFO("The robot failed to move for some reason");

  // Handle ROS communication events
  ros::spin();

  return 0;
}
