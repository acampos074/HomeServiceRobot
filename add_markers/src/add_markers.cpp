#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/Bool.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>

visualization_msgs::Marker marker;
bool picking_up_package;
ros::Publisher marker_pub;
ros::Subscriber marker_sub,marker_state_sub;
std::vector<double> marker_position{0,0};
std::vector<double> robot_current_position{0,0};

// This callback function reads amcl pose values and handles the visual and location of the marker
void check_pose_callback(const geometry_msgs::PoseWithCovarianceStamped msg)
{
  // Define a tolerance threshold to compare the distance betwen the robot and the marker
  double tolerance = 0.25;
  // Get robot current positon
  robot_current_position[0] = msg.pose.pose.position.x;
  robot_current_position[1] = msg.pose.pose.position.y;
  //ROS_INFO("Robot Position - X:%1.2f, Y:%1.2f", (float)robot_current_position[0], (float)robot_current_position[1]);
  // Get marker location
  marker_position[0] = marker.pose.position.x;
  marker_position[1] = marker.pose.position.y;

  //ROS_INFO("Marker position - X:%1.2f, Y:%1.2f", (float)marker_position[0], (float)marker_position[1]);
  //ROS_INFO("Marker position - dX:%1.2f, dY:%1.2f", (float)fabs(robot_current_position[0] - marker_position[0]), (float)fabs(robot_current_position[1] - marker_position[1]));

  // If robot location is equal to marker's current location
  if(fabs(robot_current_position[0] - marker_position[0])<tolerance && fabs(robot_current_position[1] - marker_position[1])<tolerance)
  {
    // If the package is in the pickup zone
    if(picking_up_package == true)
    {
      ROS_INFO("Picking package");
      // Hide the package by making it transparent
      marker.color.a = 0.0;
      // Change the location of the marker to its drop off location
      marker.pose.position.x = -5.85;
      marker.pose.position.y = -5.55;
      sleep(1);
      marker_pub.publish(marker);
    }
    // The package is in the drop off location
    else{
      ROS_INFO("Dropping package");
      // Make the package visible
      marker.color.a = 1.0;
      sleep(1);
      marker_pub.publish(marker);
    }
  }
}

void marker_state_callback(const std_msgs::Bool state)
{
  ROS_INFO("STATE CALLBACK");
  // Update the package state
  picking_up_package = state.data;
}

int main( int argc, char** argv )
{
  // TODO: edit node name to add_markers
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  // subscribe node to the amcl pose to keep track of the robot's pose inside the check_pose_callback function
  marker_sub = n.subscribe("/amcl_pose",10,check_pose_callback);
  marker_state_sub = n.subscribe("/pick_objects/marker_state",10,marker_state_callback);

  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "basic_shapes";
  marker.id = 0;

  // Set the marker type
  marker.type = visualization_msgs::Marker::CUBE;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = 6.11;
  marker.pose.position.y = -3.81;
  marker.pose.position.z = 0.125;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.25;
  marker.scale.y = 0.25;
  marker.scale.z = 0.25;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration(); // never auto-delete the object
  picking_up_package = true;
  // Publish the marker at the pickup zone
  sleep(1);
  marker_pub.publish(marker);

  // Handle ROS communication events
  ros::spin();

  return 0;

}
