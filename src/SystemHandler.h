#ifndef SYSTEM_HANDLER_H
#define SYSTEM_HANDLER_H

#include "string.h"
#include "OdeSolver.h"
#include "vector3d.h"
#include "spring_boxes/UnityUpdate.h"
#include "spring_boxes/ObjectState.h"

// Handles the system state to pass to the object
class SystemHandler {
  private:
    std::string id;
    OdeSolver solver;
    bool initialised = false, system_valid = false;

    // Parameters for spring connection
    float k;  // spring constant
    float c;  // damper constant
    float L;  // equilibrium spring length
    float attached_x, attached_y, attached_z;
    float attached_vx, attached_vy, attached_vz;
    
    void set_params(float spring_constant, float damper_constant, float equil_spring_length);

    vector3d get_attached_displacement();
    vector3d get_attached_velocity();
  public:
    SystemHandler(std::string objId);
    void unity_update_callback(const spring_boxes::UnityUpdate::ConstPtr& msg);
    void sim_update_callback(const spring_boxes::ObjectState::ConstPtr& msg);
    bool is_initialised();  // Check whether self has been initialised
    bool is_system_valid();  // Check whether all other network dependencies are initialised

    void step_solver(float timeDelta);
    spring_boxes::ObjectState generate_object_state_msg();
};

#endif