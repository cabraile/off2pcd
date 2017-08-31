#ifndef __GUARD_LINALG__
#define __GUARD_LINALG__
#include<iostream>
#include<vector>
#include<cmath>
#include<limits>

namespace linalg
{

  class Vec3D
  {

  private:
    std::vector<double> v;

  public:

    Vec3D()
    {
      v.resize(3);
      v[0] = 0; v[1] = 0; v[2] = 0;
      return ;
    }

    Vec3D(double val)
    {
      v.resize(3);
      v[0] = val; v[1] = val; v[2] = val;
      return ;
    }

    Vec3D(double _x, double _y, double _z)
    {
      v.resize(3);
      v[0] = _x;
      v[1] = _y;
      v[2] = _z;
      return ;
    }

    double& operator[](const std::size_t i){
      return v[i];
    }

    Vec3D operator+(Vec3D u)
    {
      Vec3D result(0,0,0);
      result[0] = u[0] + v[0];
      result[1] = u[1] + v[1];
      result[2] = u[2] + v[2];
      return result;
    }

    Vec3D operator-(Vec3D u)
    {
      Vec3D result(0,0,0);
      result[0] = v[0] - u[0];
      result[1] = v[1] - u[1];
      result[2] = v[2] - u[2];
      return result;
    }

    Vec3D operator-()
    {
      Vec3D result(0,0,0);
      result[0] = -v[0];
      result[1] = -v[1];
      result[2] = -v[2];
      return result;
    }

    Vec3D operator=(Vec3D u)
    {
      v[0] = u[0];
      v[1] = u[1];
      v[2] = u[2];
      return *this;
    }

    void print()
    {
      std::cout << "(" << v[0] << "," << v[1] << "," << v[2] << ")" << std::endl;
    }

    double norm()
    {
      return sqrt(pow(v[0],2) + pow(v[1],2) + pow(v[2],2));
    }

  };

  double dot(Vec3D u, Vec3D v)
  {
    return u[0] * v[0] + u[1] * v[1] + u[2] * v[2];
  }

  double angle(Vec3D u, Vec3D v)
  {
    if(u.norm() != 0 && v.norm() != 0)
    {
      double cos_angle = dot(u,v)/(u.norm() * v.norm());
      return acos(cos_angle);
    }
    return std::numeric_limits<double>::infinity();
  }

  Vec3D cross(Vec3D u, Vec3D v)
  {
    Vec3D r(u[1] * v[2] - u[2] * v[1], u[2]* v[0] - u[0] * v[2], u[0]*v[1] - u[1] * v[0]);
    return r;
  }
}

#endif
