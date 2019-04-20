#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

move_base_msgs::MoveBaseGoal sendgoal(const std::string& map, double x, double y, double w);

move_base_msgs::MoveBaseGoal sendgoal(const std::string& map, double x, double y, double w) {
  move_base_msgs::MoveBaseGoal goal;
    // set up the frame parameters
  goal.target_pose.header.frame_id = map;
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation.w = w;

  return goal;
}

int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal1 = sendgoal("map", 2.0, 1.0, 1.0);

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goal1);

  // Wait an infinite time for the results
  ac.waitForResult();

    // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Hooray, the base reached the first zone");
    ros::Duration(5).sleep();
  }
  else {
    ROS_INFO("The base failed to reach the first zone");
    ros::Duration(5).sleep();
  }

  // Second goal back to origin
  move_base_msgs::MoveBaseGoal goal2 = sendgoal("map", 0.0, 0.0, 1.0);
   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goal2);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Hooray, the base reached the second zone");
    ros::Duration(5).sleep();
  }
  else {
    ROS_INFO("The base failed to reached the second zone");
    ros::Duration(5).sleep();
  }
  return 0;
}