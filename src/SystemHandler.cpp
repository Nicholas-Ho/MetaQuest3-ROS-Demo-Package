#include "ros/ros.h"
#include "SystemHandler.h"
#include "OdeSolver.h"
#include "vector3d.h"
#include "spring_boxes/ObjectState.h"
#include "spring_boxes/UnityUpdate.h"
#include "string.h"

SystemHandler::SystemHandler(std::string objId) {
    id = objId;
    solver = OdeSolver();
}

void SystemHandler::unity_update_callback(const spring_boxes::UnityUpdate::ConstPtr& msg) {
    switch(msg->simState) {
    case 0:
        // Initialise
        for (auto objData : msg->objDataList) {
            if (objData.id == id) {
                solver.initialise(vector3d(objData.position),
                                           objData.mass);
                initialised = true;
                ROS_INFO("System is initialised.");
            }
        }
        if (!initialised) ROS_ERROR("Object Id is not found in UnityUpdate message.");
        set_params(msg->systemParams.spring_constant,
                   msg->systemParams.damper_constant,
                   msg->systemParams.equil_spring_length);
        break;

    case 1:
        // Running

        // Set system parameters
        set_params(msg->systemParams.spring_constant,
                   msg->systemParams.damper_constant,
                   msg->systemParams.equil_spring_length);

        // If the box is grabbed (BoxData.update == false), silet update only
        {
            bool found = false;
            for (auto objData : msg->objDataList) {
                if (objData.id == id) {
                    found = true;
                    solver.set_step_status(objData.update);
                    if (!objData.update) {
                        solver.silent_update(vector3d(objData.position), msg->timeDelta);
                    }
                }
            }
            if (!found) ROS_ERROR("Object Id is not found in UnityUpdate message.");
        }
        break;

    case 2:
        // Terminating
        initialised = false;
        system_valid = false;
        ROS_INFO("System terminated.");
        break;

    default:
        ROS_INFO("Invalid simState");
    }
}

void SystemHandler::sim_update_callback(const spring_boxes::ObjectState::ConstPtr& msg) {
    attached_x = msg->position.x;
    attached_y = msg->position.y;
    attached_z = msg->position.z;
    attached_vx = msg->velocity.x;
    attached_vy = msg->velocity.y;
    attached_vz = msg->velocity.z;
    system_valid = true;
}

bool SystemHandler::is_initialised() { return initialised; }
bool SystemHandler::is_system_valid() { return system_valid; }

void SystemHandler::set_params(float spring_constant, float damper_constant, float equil_spring_length) {
    k = spring_constant;
    c = damper_constant;
    L = equil_spring_length;
}

vector3d SystemHandler::get_attached_displacement() {
    return vector3d(attached_x,
                    attached_y,
                    attached_z);
}

vector3d SystemHandler::get_attached_velocity() {
    return vector3d(attached_vx,
                    attached_vy,
                    attached_vz);
}

void SystemHandler::step_solver(float timeDelta) {
    solver.step(get_attached_displacement(),
                get_attached_velocity(),
                k,
                c,
                L,
                timeDelta);
}

spring_boxes::ObjectState SystemHandler::generate_object_state_msg() {
    spring_boxes::ObjectState msg;
    msg.position = solver.get_displacement().to_point_msg();
    msg.velocity = solver.get_velocity().to_vec3_msg();
    return msg;
}