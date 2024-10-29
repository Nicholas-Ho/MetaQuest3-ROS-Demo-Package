#include "ros/ros.h"
#include "string.h"
#include "spring_boxes/ObjectState.h"
#include "vector3d.h"
#include "OdeSolver.h"
#include "SystemHandler.h"

int FREQUENCY = 100;  // Hz

int main(int argc, char **argv)
{
  // Set up ROS
  ros::init(argc, argv, "box_simulator");
  ros::NodeHandle n;

  // Get ID
  std::string node_id;
  n.getParam("node_id", node_id);

  // Initialise key classes
  SystemHandler systemHandler = SystemHandler(argv[1]);  // ID to override

  // Register subscribers and publishers
  ros::Subscriber unity_sub = n.subscribe("unity_updates", 1000, &SystemHandler::unity_update_callback, &systemHandler);
  ros::Subscriber connected_sub = n.subscribe("box_state_attached", 1000, &SystemHandler::sim_update_callback, &systemHandler);  // Topic to override
  ros::Publisher pub = n.advertise<spring_boxes::ObjectState>("box_state_self", 1000);  // Topic to override

  ros::Rate loop_rate(FREQUENCY);  // Hz
  float timeDelta = 1 / (float)FREQUENCY;
  ROS_INFO("Period: %fs", timeDelta);

  ROS_INFO("Node is running");

  while (ros::ok())
  {
    if (systemHandler.is_initialised()) {
      if(systemHandler.is_system_valid()) systemHandler.step_solver(timeDelta);
      pub.publish(systemHandler.generate_object_state_msg());
    }

    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}