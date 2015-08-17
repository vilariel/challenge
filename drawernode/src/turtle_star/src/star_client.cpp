#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <turtle_actionlib/ShapeAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_shape"); 

  if (argc != 3)
  {
    ROS_INFO("Usage: star_client edges radius");
    return -1;
  }
  
  // create the action client
  // true causes the client to spin it's own thread
  actionlib::SimpleActionClient<turtle_actionlib::ShapeAction> ac("turtle_shape", true); 

  ROS_ERROR("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time
 
  // send a goal to the action 
  turtle_actionlib::ShapeGoal goal;
  goal.edges = atoi(argv[1]);
  goal.radius = atof(argv[2]);
  ac.sendGoal(goal);
  ROS_INFO("Action server started, sending goal (edges: %d, radius: %f)", goal.edges, goal.radius);
  
  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(60.0));

  if (finished_before_timeout)
    {
      actionlib::SimpleClientGoalState state = ac.getState();
      ROS_INFO("Action finished: %s",state.toString().c_str());
    }
  else  
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}
