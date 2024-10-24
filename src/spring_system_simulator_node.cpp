#include "ros/ros.h"
#include "spring_boxes/SpringSystemState.h"
#include "vector3d.h"
#include "OdeSolver.h"
#include "SystemHandler.h"

int FREQUENCY = 100;  // Hz

int main(int argc, char **argv)
{
  // Initialise key classes
  OdeSolver solver1 = OdeSolver(), solver2 = OdeSolver();
  SystemHandler systemHandler = SystemHandler(solver1, solver2);

  // Set up ROS
  ros::init(argc, argv, "spring_system_simulator");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("unity_updates", 1000, &SystemHandler::unity_update_callback, &systemHandler);
  ros::Publisher pub = n.advertise<spring_boxes::SpringSystemState>("spring_system_state", 1000);

  ros::Rate loop_rate(FREQUENCY);  // Hz
  float timeDelta = 1 / FREQUENCY;


  while (ros::ok())
  {
    if (systemHandler.is_initialised()) {
      spring_boxes::SpringSystemState msg;

      // Get values required for simulation
      vector3d obj1Pos = solver1.get_displacement();
      vector3d obj2Pos = solver2.get_displacement();
      vector3d obj1Vel = solver1.get_velocity();
      vector3d obj2Vel = solver2.get_velocity();

      // Semi-explicit Euler: update velocity then displacement
      solver1.step_velocity(obj2Pos, obj2Vel, timeDelta);
      solver2.step_velocity(obj1Pos, obj2Vel, timeDelta);
      msg.obj1final = solver1.step_displacement(timeDelta).to_point_msg();
      msg.obj2final = solver2.step_displacement(timeDelta).to_point_msg();

      // Publish state
      pub.publish(msg);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}