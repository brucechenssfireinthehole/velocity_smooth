#include <iostream>
#include <fstream>
#include <iomanip>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <velocity_smooth/smooth_srv.h>

bool test(velocity_smooth::smooth_srv::Request &req,
              velocity_smooth::smooth_srv::Response &res)
{
  int n;
  n = 0;
  ros::Rate r(100);
  double time;
  ros::Time begin = ros::Time::now();
   while (ros::ok())
  {
    ros::spinOnce();
    ros::Duration curr = ros::Time::now() - begin;
    time = curr.toSec();
    n++;
    if (n == 1000)
    break;
    r.sleep();
  }
  ros::Duration(1).sleep();
  res.mark = req.x+req.y;
  return true;
}


int main(int argc, char **argv)
{	
   ros::init(argc, argv, "test_srv");
   ros::ServiceServer service;
   ros::NodeHandle node;
   service = node.advertiseService("/test_srv", test);
   ros::spin();
   return 0;
}
