#include "ros/ros.h"
#include "geometry_msgs/Point.h"

void subscriberCallback(const geometry_msgs::Point::ConstPtr& msg)
{
  ROS_INFO("I heard: {%f, %f, %f}", msg->x, msg->y, msg->z);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "subscriber");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("position", 1000, subscriberCallback);

  ros::spin();

  return 0;
}
