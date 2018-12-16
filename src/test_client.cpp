#include <ros/ros.h>
#include <velocity_smooth/smooth_srv.h>
#include <velocity_smooth/smooth_srv_multi.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>
#include <map>

using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_client");
    ros::NodeHandle node;
    ros::ServiceClient client = node.serviceClient<velocity_smooth::smooth_srv>("/test_srv"); 
    velocity_smooth::smooth_srv srv;
    srv.request.x = 2.0;
    srv.request.y = 3.0;
    srv.request.theta =1.0;
   

      sleep(1);
    if(client.call(srv))
    {
        ROS_INFO("success");
      ROS_INFO("the response is %ld", (long int)srv.response.mark);
    }
    else
    {
        ROS_ERROR("Failed");
    }

    return 0;
}
