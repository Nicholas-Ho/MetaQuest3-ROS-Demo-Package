#ifndef ODE_SOLVER_H
#define ODE_SOLVER_H

#include "vector3d.h"

class OdeSolver {
  private:
    float x, y, z;
    float vx, vy, vz;
    float step_status = false;  // only step if true
    float k;  // spring constant
    float c;  // damper constant
    float L;  // equilibrium spring length
    float mass;
  public:
    void initialise(vector3d vec,
                    float spring_constant,
                    float damper_constant,
                    float equil_spring_length,
                    float obj_mass);

    float calc_acceleration(float objDisp,
                            float objVel,
                            float attachedDisp,
                            float attachedVel,
                            float equil_spring_length);

    void step_velocity(vector3d attachedPos,
                       vector3d attachedVel,
                       float timeDelta);

    vector3d step_displacement(float timeDelta);

    void step(vector3d attachedPos,
              vector3d attachedVel,
              float timeDelta);

    void set_step_status(bool status);
    void set_params(float spring_constant, float damper_constant, float equil_spring_length, float obj_mass);
    void silent_update(vector3d pos, float timeDelta);

    vector3d get_velocity();
    vector3d get_displacement();
};

#endif