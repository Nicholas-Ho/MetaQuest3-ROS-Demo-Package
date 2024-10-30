#include "OdeSolver.h"
#include "vector3d.h"
#include "ros/ros.h"


void OdeSolver::initialise(vector3d vec,
                           float obj_mass) {
    x = vec.x; y = vec.y; z = vec.z;
    vx = 0, vy = 0, vz = 0;
    mass = obj_mass;
    step_status = true;  // Allow simulation
}

float OdeSolver::calc_acceleration(float objDisp,
                                   float objVel,
                                   float attachedDisp,
                                   float attachedVel,
                                   float k,  // spring constant
                                   float c,  // damper constant
                                   float equil_spring_length) {
    return (k * (attachedDisp - objDisp - equil_spring_length) + c * (attachedVel - objVel)) / mass;
}

void OdeSolver::step_velocity(vector3d attachedPos,
                              vector3d attachedVel,
                              float k,  // spring constant
                              float c,  // damper constant
                              float L,  // equilibrium spring length
                              float timeDelta) {
    if (!step_status) return ;  // Only continue if (step_status == true)
    vector3d spring_dir = subtract_vector(attachedPos, get_displacement());
    spring_dir = multiply_vector(spring_dir, L / spring_dir.magnitude());
    vx = vx + calc_acceleration(x, vx, attachedPos.x, attachedVel.x, k, c, spring_dir.x) * timeDelta;
    vy = vy + calc_acceleration(y, vy, attachedPos.y, attachedVel.y, k, c, spring_dir.y) * timeDelta;
    vz = vz + calc_acceleration(z, vz, attachedPos.z, attachedVel.z, k, c, spring_dir.z) * timeDelta;
}

void OdeSolver::step_displacement(float timeDelta) {
    if (!step_status) return ;  // Only continue if (step_status == true)
    x = x + vx * timeDelta;
    y = y + vy * timeDelta;
    z = z + vz * timeDelta;
}

void OdeSolver::step(vector3d attachedPos,
                     vector3d attachedVel,
                     float k,  // spring constant
                     float c,  // damper constant
                     float L,  // equilibrium spring length
                     float timeDelta) {
    // Semi-explicit Euler: update velocity then displacement
    step_velocity(attachedPos, attachedVel, k, c, L, timeDelta);
    step_displacement(timeDelta);
}

void OdeSolver::set_step_status(bool status) { step_status = status; }

void OdeSolver::silent_update(vector3d pos, float timeDelta) {
    vx = (pos.x - x) / timeDelta;
    vy = (pos.y - y) / timeDelta;
    vz = (pos.z - z) / timeDelta;
    x = pos.x; y = pos.y; z = pos.z;
}

vector3d OdeSolver::get_velocity() { return vector3d(vx, vy, vz); }
vector3d OdeSolver::get_displacement() { return vector3d(x, y, z); }