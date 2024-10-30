#include "vector3d.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"
#include <cmath>

// vector3d definitions
vector3d::vector3d(float iX, float iY, float iZ) : x(iX), y(iY), z(iZ) {}
vector3d::vector3d(geometry_msgs::Point msg) : x(msg.x), y(msg.y), z(msg.z) {}
vector3d::vector3d(geometry_msgs::Vector3 msg) : x(msg.x), y(msg.y), z(msg.z) {}

geometry_msgs::Point vector3d::to_point_msg() {
  geometry_msgs::Point point = geometry_msgs::Point();
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

geometry_msgs::Vector3 vector3d::to_vec3_msg() {
  geometry_msgs::Vector3 vec = geometry_msgs::Vector3();
  vec.x = x;
  vec.y = y;
  vec.z = z;
  return vec;
}

float vector3d::magnitude() { return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2)); }

// Related functions

vector3d subtract_vector(vector3d a, vector3d b) {
  // a - b
  return vector3d(a.x - b.x, a.y - b.y, a.z - b.z);
}

vector3d multiply_vector(vector3d a, float k) {
  // a * k
  return vector3d(a.x * k, a.y * k, a.z * k);
}