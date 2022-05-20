#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal pickUp;
  move_base_msgs::MoveBaseGoal dropGoal;

  // Pick up
  // set up the frame parameters
  pickUp.target_pose.header.frame_id = "map";
  pickUp.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  pickUp.target_pose.pose.position.x = 0.0;
  pickUp.target_pose.pose.position.y = -1.0;
  pickUp.target_pose.pose.orientation.w = 2.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Picking up");
  ac.sendGoal(pickUp);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    // pause 5 sec
    ros::Duration(5.0).sleep();

    // set up the frame parameters
    dropGoal.target_pose.header.frame_id = "map";
    dropGoal.target_pose.header.stamp = ros::Time::now();

    // Define a position and orientation for the robot to reach
    dropGoal.target_pose.pose.position.x = -1.5;
    dropGoal.target_pose.pose.position.y = 1.5;
    dropGoal.target_pose.pose.orientation.w = 2.0;

    ROS_INFO("Dropping");
    ac.sendGoal(dropGoal);

    // Wait an infinite time for the results
    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Mission complete");
    else
        ROS_INFO("Not drop");
  }
  else
    ROS_INFO("Not pick up");

  return 0;
}