#ifndef __GUARD_CG__

#define __GUARD_CG__

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "linalg.hpp"

namespace cg
{

  struct vertex
  {
    float x;
    float y;
    float z;
  };

  struct facade
  {
    int v1;
    int v2;
    int v3;
  };

  class Mesh
  {
  private:
    std::vector<vertex> vertices;
    std::vector<facade> facades;
    vertex max;
    vertex min;
    bool redundant;

    // Adds points in the pointcloud without redundancy, if not reduntant cloud.
    // Returns whether the point was added or not
    // It is not added if a point already exists at that position
    bool add2PC(
      pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
      pcl::PointXYZ p
    )
    {
      if(!redundant)
      {
        for(pcl::PointXYZ q : cloud->points)
        {
          if(p.x == q.x && p.y == q.y && p.z == q.z)
            return false;
        }
      }
      cloud->points.push_back(p);
      return true;
    }

    // TODO: FIX
    // > Adapted from the ray tracing algorithm of:
    // https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-rendering-a-triangle/barycentric-coordinates
    // https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-rendering-a-triangle/ray-triangle-intersection-geometric-solution
    // access 30/08/2017.
    bool belong2boundaries(
      vertex p,
      vertex p_A,
      vertex p_B,
      vertex p_C
    )
    {
      linalg::Vec3D v_A(p_A.x, p_A.y, p_A.z);
      linalg::Vec3D v_B(p_B.x, p_B.y, p_B.z);
      linalg::Vec3D v_C(p_C.x, p_C.y, p_C.z);
      linalg::Vec3D P(p.x, p.y, p.z);
      double u, v, w;

      linalg::Vec3D AB = v_B - v_A;
      linalg::Vec3D AC = v_C - v_A;
      linalg::Vec3D BC = v_C - v_B;

      linalg::Vec3D PA = v_A - P;
      linalg::Vec3D PB = v_B - P;
      linalg::Vec3D PC = v_C - P;

      double tri_abc_area = linalg::cross(AB, AC).norm();
      double tri_cap_area = linalg::cross(PA, PC).norm();
      double tri_abp_area = linalg::cross(PA, PB).norm();
      double tri_bcp_area = linalg::cross(PB, PC).norm();

      u = tri_cap_area/tri_abc_area;
      v = tri_abp_area/tri_abc_area;
      w = tri_bcp_area/tri_abc_area;

      if( 0 < u && u < 1 &&
          0 < v && v < 1 &&
          0 < w && w < 1)
          return true;

      return false;
    }

    double minimum(double x1, double x2, double x3)
    {
      double min = (x1 < x2) ? x1 : x2;
      min = (min < x3) ? min : x3;
      return min;
    }

    double maximum(double x1, double x2, double x3)
    {
      double max = (x1 > x2) ? x1 : x2;
      max = (max > x3) ? max : x3;
      return max;
    }

    // > TODO: FIX
    // > Fill the polygon with 3 vertices using the ray trace algorithm
    void fill_polygon(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
      vertex v1,
      vertex v2,
      vertex v3,
      double step_size
    )
    {
      pcl::PointXYZ p_min;
      pcl::PointXYZ p_max;
      p_min.x = minimum(v1.x, v2.x, v3.x);
      p_min.y = minimum(v1.y, v2.y, v3.y);
      p_min.z = minimum(v1.z, v2.z, v3.z);
      p_max.x = maximum(v1.x, v2.x, v3.x);
      p_max.y = maximum(v1.y, v2.y, v3.y);
      p_max.z = maximum(v1.z, v2.z, v3.z);

      for(double x = p_min.x + step_size; x < p_max.x; x+= step_size)
      {
        for(double y = p_min.x + step_size; y < p_max.y; y+= step_size)
        {
          for(double z = p_min.z + step_size; z < p_max.z; z+= step_size)
          {
            vertex v;
            v.x = x; v.y = y; v.z = z;
            if(belong2boundaries(v, v1, v2, v3))
            {
              pcl::PointXYZ p;
              p.x = x; p.y = y; p.z = z;
              add2PC(cloud, p);
            }
          }
        }
      }

      return ;
    }

    void fill_lines(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
      vertex v1,
      vertex v2,
      double step_size
    )
    {
      for(double t = step_size; t < 1; t += step_size)
      {
        pcl::PointXYZ p;
        p.x = v2.x + (v1.x - v2.x) * t;
        p.y = v2.y + (v1.y - v2.y) * t;
        p.z = v2.z + (v1.z - v2.z) * t;
        add2PC(cloud, p);
      }
      return ;
    }

  public:
    Mesh(
      std::vector<vertex> _vertices,
      std::vector<facade> _facades
    )
    {
      vertices = _vertices;
      facades = _facades;
      redundant = true;
      return ;
    }

    void set_redundancy(bool value) { redundant = value; }

    void get_extrema(
      vertex &max,
      vertex &min
    )
    {

      max.x = std::numeric_limits<double>::min();
      max.y = std::numeric_limits<double>::min();
      max.z = std::numeric_limits<double>::min();
      min.x = std::numeric_limits<double>::max();
      min.y = std::numeric_limits<double>::max();
      min.z = std::numeric_limits<double>::max();
      // > Get max - min
      for(vertex v : vertices)
      {
        max.x = (max.x < v.x) ? v.x : max.x;
        max.y = (max.y < v.y) ? v.y : max.y;
        max.z = (max.z < v.z) ? v.z : max.z;
        min.x = (min.x > v.x) ? v.x : min.x;
        min.y = (min.y > v.y) ? v.y : min.y;
        min.z = (min.z > v.z) ? v.z : min.z;
      }
    }

    void get_average(vertex & avg)
    {
      for(vertex v : vertices)
      {
        avg.x += v.x;
        avg.y += v.y;
        avg.z += v.z;
      }
      avg.x /= vertices.size();
      avg.y /= vertices.size();
      avg.z /= vertices.size();
      return ;
    }

    void centralize()
    {
      vertex avg;
      get_average(avg);
      for(vertex & v : vertices)
      {
        v.x -= avg.x;
        v.y -= avg.y;
        v.z -= avg.z;
      }
      return ;
    }

    // > Points' coordinates range from 0 to 1
    void normalize()
    {
      vertex max, min;
      get_extrema(max, min);
      // > Normalize
      for(vertex & v : vertices)
      {
        v.x = (v.x - min.x) / (max.x - min.x);
        v.y = (v.y - min.y) / (max.y - min.y);
        v.z = (v.z - min.z) / (max.z - min.z);
      }
      return ;
    }

    // > Generates a PointCloud
    void to_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
      double step_size=0.1
    )
    {
      cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr
          (new pcl::PointCloud<pcl::PointXYZ>);
      // > Store vertices
      for(vertex v : vertices)
      {
        pcl::PointXYZ p;
        p.x = v.x; p.y = v.y; p.z = v.z;
        add2PC(cloud, p);
      }

      // > Computes points from the edges for each face
      for(facade f : facades)
      {
        fill_lines(cloud, vertices[f.v1], vertices[f.v2], step_size);
        fill_lines(cloud, vertices[f.v1], vertices[f.v3], step_size);
        fill_lines(cloud, vertices[f.v2], vertices[f.v3], step_size);
        //fill_polygon(cloud, vertices[f.v1], vertices[f.v2],
        //    vertices[f.v3], step_size);
      }

      return ;
    }

  };
}

#endif
