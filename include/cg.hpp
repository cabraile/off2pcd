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

    void fill_polygon(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
      const vertex v1,
      const vertex v2,
      const vertex v3,
      const double step_size
    )
    {
      std::vector<pcl::PointXYZ> edge_points;

      vertex a = v1;
      vertex b = v2;
      vertex c = v3;

      // > Step 1 : compute the line points between a and b
      for(double t = step_size; t < 1; t += step_size)
      {
        pcl::PointXYZ p;
        p.x = b.x + (a.x - b.x) * t;
        p.y = b.y + (a.y - b.y) * t;
        p.z = b.z + (a.z - b.z) * t;
        edge_points.push_back(p);
      }

      // > Step 2 : compute the line points between each edge point and c
      for(pcl::PointXYZ p : edge_points)
      {
        for(double t = step_size; t < 1; t += step_size)
        {
          pcl::PointXYZ q;
          q.x = c.x + (p.x - c.x) * t;
          q.y = c.y + (p.y - c.y) * t;
          q.z = c.z + (p.z - c.z) * t;
          add2PC(cloud, q);
        }
      }

      return ;
    }

    void fill_lines(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
      const vertex v1,
      const vertex v2,
      const double step_size
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

    // > Points' coordinates range from -1 to 1
    void normalize(double a = -1.0, double b = 1.0)
    {
      vertex max, min;
      get_extrema(max, min);
      // > Normalize
      for(vertex & v : vertices)
      {
        v.x = a + (b - a) * (v.x - min.x) / (max.x - min.x);
        v.y = a + (b - a) * (v.y - min.y) / (max.y - min.y);
        v.z = a + (b - a) * (v.z - min.z) / (max.z - min.z);
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
        fill_polygon(cloud, vertices[f.v1], vertices[f.v2],
            vertices[f.v3], step_size);
      }

      return ;
    }

    void add_vertices_to_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
    {
      for(vertex v : vertices)
      {
        pcl::PointXYZ p;
        p.x = v.x; p.y = v.y; p.z = v.z;
        add2PC(cloud, p);
      }
      return ;
    }

    void add_lines_to_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
      double step_size=0.1)
    {
      for(facade f : facades)
      {
        fill_lines(cloud, vertices[f.v1], vertices[f.v2], step_size);
        fill_lines(cloud, vertices[f.v1], vertices[f.v3], step_size);
        fill_lines(cloud, vertices[f.v2], vertices[f.v3], step_size);
      }
      return ;
    }
    void sample_facades_to_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
      double step_size=0.1)
    {
      for(facade f : facades)
        fill_polygon(cloud, vertices[f.v1], vertices[f.v2],
            vertices[f.v3], step_size);
    }

  };
}

#endif
