#include "ros/ros.h"
#include "spring_boxes/SpringOdeSolver.h"
#include "geometry_msgs/Point.h"
#include <cmath>

struct vector3d {
  float x, y, z;

  vector3d(float iX, float iY, float iZ) : x(iX), y(iY), z(iZ) {}
  vector3d(geometry_msgs::Point msg) : x(msg.x), y(msg.y), z(msg.z) {}

  geometry_msgs::Point to_point_msg() {
    geometry_msgs::Point point = geometry_msgs::Point();
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
  }
  float magnitude() { return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2)); }
};

vector3d subtract_vector(vector3d a, vector3d b) {
  // a - b
  return vector3d(a.x - b.x, a.y - b.y, a.z - b.z);
}

vector3d multiply_vector(vector3d a, float k) {
  // a * k
  return vector3d(a.x * k, a.y * k, a.z * k);
}

class OdeSolver {
  private:
    float x, y, z;
    float vx = 0, vy = 0, vz = 0;
  public:
    OdeSolver(float iX, float iY, float iZ) : x(iX), y(iY), z(iZ) {}

    float calc_acceleration(float objDisp,
                            float objVel,
                            float attachedDisp,
                            float attachedVel,
                            float k,  // spring constant
                            float c,  // damper constant
                            float L,  // equilibrium spring length
                            float mass) {
      return (k * (attachedDisp - objDisp - L) + c * (attachedVel - objVel)) / mass;
    }

    void step_velocity(vector3d attachedPos,
                       vector3d attachedVel,
                       float k,  // spring constant
                       float c,  // damper constant
                       float L,  // equilibrium spring length
                       float mass,
                       float timeDelta) {
      vector3d spring_dir = subtract_vector(attachedPos, get_displacement());
      spring_dir = multiply_vector(spring_dir, L / spring_dir.magnitude());
      vx = vx + calc_acceleration(x, vx, attachedPos.x, attachedVel.x, k, c, spring_dir.x, mass) * timeDelta;
      vy = vy + calc_acceleration(y, vy, attachedPos.y, attachedVel.y, k, c, spring_dir.y, mass) * timeDelta;
      vz = vz + calc_acceleration(z, vz, attachedPos.z, attachedVel.z, k, c, spring_dir.z, mass) * timeDelta;
    }

    vector3d step_displacement(float timeDelta) {
      x = x + vx * timeDelta;
      y = y + vy * timeDelta;
      z = z + vz * timeDelta;
      return get_displacement();
    }

    void step(vector3d attachedPos,
              vector3d attachedVel,
              float k,  // spring constant
              float c,  // damper constant
              float L,  // equilibrium spring length
              float mass,
              float timeDelta) {
      step_velocity(attachedPos, attachedVel, k, c, L, mass, timeDelta);
      step_displacement(timeDelta);
    }

    void silent_update(vector3d pos, float timeDelta) {
      vx = (pos.x - x) / timeDelta;
      vy = (pos.y - y) / timeDelta;
      vz = (pos.z - z) / timeDelta;
      x = pos.x; y = pos.y; z = pos.z;
    }

    vector3d get_velocity() { return vector3d(vx, vy, vz); }
    vector3d get_displacement() { return vector3d(x, y, z); }
};

OdeSolver box1Solver = OdeSolver(0,0,0), box2Solver = OdeSolver(0,0,0);
bool running = false;

bool solve(spring_boxes::SpringOdeSolver::Request  &req,
         spring_boxes::SpringOdeSolver::Response &res)
{
  if (req.simState == 0) {  // Reset
    box1Solver = OdeSolver(req.obj1initial.x, req.obj1initial.y, req.obj1initial.z);
    box2Solver = OdeSolver(req.obj2initial.x, req.obj2initial.y, req.obj2initial.z);
    res.obj1final = req.obj1initial;
    res.obj2final = req.obj2initial;
    running = true;
    return true;
  } else if (req.simState == 2) {  // Termination
    running = false;
    return true;
  }

  if (!running) {
    res.obj1final = req.obj1initial;
    res.obj2final = req.obj2initial;
    return true;
  }

  // If no update, silently update solvers. Else, step through the solver.
  vector3d obj1Pos = req.obj1update ? box1Solver.get_displacement() : vector3d(req.obj1initial);
  vector3d obj2Pos = req.obj2update ? box2Solver.get_displacement() : vector3d(req.obj2initial);
  vector3d obj1Vel = box1Solver.get_velocity();
  vector3d obj2Vel = box2Solver.get_velocity();

  // Silent updates
  if (!req.obj1update) {
    ROS_INFO("Grabbed box 1");
    box1Solver.silent_update(obj1Pos, req.timeDelta);
    res.obj1final = req.obj1initial;
  }

  if (!req.obj2update) {
    ROS_INFO("Grabbed box 2");
    box2Solver.silent_update(obj2Pos, req.timeDelta);
    res.obj2final = req.obj2initial;
  }

  // Velocity
  if (req.obj1update) {
    box1Solver.step_velocity(obj2Pos,
                             obj2Vel,
                             req.spring_constant,
                             req.damper_constant,
                             req.equil_spring_length,
                             req.obj1mass,
                             req.timeDelta);
  }

  if (req.obj2update) {
    box2Solver.step_velocity(obj1Pos,
                             obj1Vel,
                             req.spring_constant,
                             req.damper_constant,
                             req.equil_spring_length,
                             req.obj2mass,
                             req.timeDelta);
  }
  
  // Displacement
  if (req.obj1update) {
    res.obj1final = box1Solver.step_displacement(req.timeDelta).to_point_msg();
  }

  if (req.obj2update) {
    res.obj2final = box2Solver.step_displacement(req.timeDelta).to_point_msg();
  }

  // ROS_INFO("request box 1: {%f, %f, %f}", req.obj1initial.x, req.obj1initial.y, req.obj1initial.z);
  // ROS_INFO("request box 2: {%f, %f, %f}", req.obj2initial.x, req.obj2initial.y, req.obj2initial.z);
  // ROS_INFO("sending back response: {%f, %f, %f}", res.obj1final.x, res.obj1final.y, res.obj1final.z);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "spring_ode_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("spring_ode_solver", solve);
  ROS_INFO("Spring ODE solver ready.");
  ros::spin();

  return 0;
}