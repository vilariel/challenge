#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <cmath>
#include <math.h>
#include <angles/angles.h>

#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

#include <string>
#include <sstream>
#include <vector>

double radius_, interior_angle_, side_len_;
double start_x_, start_y_, start_theta_;
double dis_error_, theta_error_;
int edges_ , edge_progress_;
bool start_edge_;
bool is_active_;
geometry_msgs::Twist command_;
ros::Subscriber sub_pose_, sub_draw_;
ros::Publisher pub_cmd_;

std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}


std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}

void startDrawing(int edges, double radius)
{
  ROS_INFO("Draw star: %d edges, %f radius", edges, radius);
  //save the goal as private variables
  edges_ = edges;
  radius_ = radius;
  // reset helper variables
  interior_angle_ = M_PI / edges_;
  //compute the side length of the polygon
  side_len_ = 2 * radius_ *  cos(interior_angle_ / 2);
  //store the result values
  edge_progress_ = 0;
  start_edge_ = true;
  is_active_ = true;
}  

void callDrawing(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
  std::vector<std::string> params = split(msg->data, ' ');
  //for(std::vector<std::string>::iterator it = x.begin(); it != x.end(); ++it) {
  //    std::cout << *it << std::endl;
  //}
  if (params.size() != 2) {
    ROS_INFO("Must be called with [edges] [radius]");
  } else {
    startDrawing(atoi(params.at(0).c_str()), atof(params.at(1).c_str()));
  }
}

void controlMovement(const turtlesim::Pose::ConstPtr& msg)
{
  // make sure that the action hasn't been canceled
  if (!is_active_) return;

  if (edge_progress_ < edges_)
  {
    // scalar values for drive the turtle faster and straighter
    double l_scale = 6.0;
    double a_scale = 6.0;
    double error_tol = 0.00001;

    if (start_edge_)
    {
      start_x_ = msg->x;
      start_y_ = msg->y;
      start_theta_ = msg->theta;
      start_edge_ = false;
    }

    // compute the distance and theta error for the shape
    dis_error_ = side_len_ - fabs(sqrt((start_x_- msg->x)*(start_x_-msg->x) + (start_y_-msg->y)*(start_y_-msg->y)));
    theta_error_ = angles::normalize_angle_positive(M_PI - interior_angle_ - (msg->theta - start_theta_));

    if (dis_error_ > error_tol)
    {
      command_.linear.x = l_scale*dis_error_;
      command_.angular.z = 0;
    }
    else if (dis_error_ < error_tol && fabs(theta_error_)> error_tol)
    { 
      command_.linear.x = 0;
      command_.angular.z = a_scale*theta_error_;
    }
    else if (dis_error_ < error_tol && fabs(theta_error_)< error_tol)
    {
      command_.linear.x = 0;
      command_.angular.z = 0;
      start_edge_ = true;
      edge_progress_++;
    }  
    else
    {
      command_.linear.x = l_scale*dis_error_;
      command_.angular.z = a_scale*theta_error_;
    } 
    // publish the velocity command
    pub_cmd_.publish(command_);

  } 
  else
  {
    ROS_INFO("Succeeded!");
    is_active_ = false;
  }   
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtle_shape");
  ros::NodeHandle nh;
  //subscribe to the data topic of interest
  sub_pose_ = nh.subscribe("/turtle1/pose", 1, controlMovement);
  pub_cmd_ = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);
  sub_draw_ = nh.subscribe("/draw", 1, callDrawing);  
  ros::spin();
  return 0;
}
