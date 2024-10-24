#ifndef VECTOR3D_H
#define VECTOR3D_H

#include "geometry_msgs/Point.h"

struct vector3d {
  float x, y, z;

  vector3d(float iX, float iY, float iZ);
  vector3d(geometry_msgs::Point msg);

  geometry_msgs::Point to_point_msg();
  float magnitude();
};

vector3d subtract_vector(vector3d a, vector3d b);

vector3d multiply_vector(vector3d a, float k);

#endif