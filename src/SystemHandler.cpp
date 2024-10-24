#include "ros/ros.h"
#include "SystemHandler.h"
#include "OdeSolver.h"
#include "spring_boxes/UnityUpdate.h"

SystemHandler::SystemHandler(OdeSolver s1, OdeSolver s2) : solver1(s1), solver2(s2) {}

void SystemHandler::unity_update_callback(const spring_boxes::UnityUpdate::ConstPtr& msg) {
    switch(msg->simState) {
    case 0:
        // Initialise
        solver1.initialise(vector3d(msg->box1data.position),
                            msg->params.spring_constant,
                            msg->params.damper_constant,
                            msg->params.equil_spring_length,
                            msg->box1data.mass);
        solver2.initialise(vector3d(msg->box2data.position),
                            msg->params.spring_constant,
                            msg->params.damper_constant,
                            msg->params.equil_spring_length,
                            msg->box2data.mass);
        initialised = true;
        break;

    case 1:
        // Running

        // Set system parameters
        solver1.set_params(msg->params.spring_constant,
                            msg->params.damper_constant,
                            msg->params.equil_spring_length,
                            msg->box1data.mass);
        solver2.set_params(msg->params.spring_constant,
                            msg->params.damper_constant,
                            msg->params.equil_spring_length,
                            msg->box2data.mass);

        // If the box is grabbed (BoxData.update == false), silet update only
        solver1.set_step_status(msg->box1data.update);
        solver2.set_step_status(msg->box2data.update);
        if (!msg->box1data.update) {
        solver1.silent_update(vector3d(msg->box1data.position), msg->timeDelta);
        }
        if (!msg->box2data.update) {
        solver2.silent_update(vector3d(msg->box2data.position), msg->timeDelta);
        }
        break;

    case 2:
        // Terminating
        initialised = false;
        break;

    default:
        ROS_INFO("Invalid simState");
    }
}

bool SystemHandler::is_initialised() { return initialised; }