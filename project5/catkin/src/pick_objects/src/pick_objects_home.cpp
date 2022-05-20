#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <string>
#include "std_msgs/String.h"

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// Publish message helper function
void publishMsg(ros::Publisher pub, std::string raw){
  std_msgs::String msg;
  msg.data = raw;
  pub.publish(msg);
}

int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");

  // Create publisher for add_markers
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<std_msgs::String>("goal_status", 1);

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // Pick up
  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Pick up pose
  goal.target_pose.pose.position.x = -1.0;
  goal.target_pose.pose.position.y = 1.0;
  goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Picking up");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){

    // Drop pose
    goal.target_pose.pose.position.x = 4.2;
    goal.target_pose.pose.position.y = 0.5;
    goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Picked, now dropping");
    publishMsg(pub, "is-picked-up");
    ac.sendGoal(goal);

    // Wait an infinite time for the results
    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("Mission complete");
        publishMsg(pub, "is-drop");
    }
    else
        ROS_INFO("Not drop");
  }
  else
    ROS_INFO("Not pick up");

  return 0;
}