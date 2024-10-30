#ifndef ODE_SOLVER_H
#define ODE_SOLVER_H

#include "vector3d.h"

// Handles the dynamics of the object given external conditions
class OdeSolver {
  private:
    float x, y, z;
    float vx, vy, vz;
    bool step_status = false;  // only step if true
    float mass;
  public:
    void initialise(vector3d vec,
                    float obj_mass);

    float calc_acceleration(float objDisp,
                            float objVel,
                            float attachedDisp,
                            float attachedVel,
                            float k,  // spring constant
                            float c,  // damper constant
                            float equil_spring_length);

    void step_velocity(vector3d attachedPos,
                       vector3d attachedVel,
                       float k,  // spring constant
                       float c,  // damper constant
                       float L,  // equilibrium spring length
                       float timeDelta);

    void step_displacement(float timeDelta);

    void step(vector3d attachedPos,
              vector3d attachedVel,
              float k,  // spring constant
              float c,  // damper constant
              float L,  // equilibrium spring length
              float timeDelta);

    void set_step_status(bool status);
    void silent_update(vector3d pos, float timeDelta);

    vector3d get_velocity();
    vector3d get_displacement();
};

#endif