#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/visualization/cloud_viewer.h>

#include "include/offparser.hpp"
#include "include/cg.hpp"

#define DEFAULT_STEP_SIZE 0.1

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> CloudT;

CloudT::Ptr transposeToMassCenter( const CloudT::Ptr & cloud )
{
  PointT centroid;
  CloudT::Ptr out_cloud(new CloudT);
  pcl::computeCentroid<PointT, PointT>(*cloud, centroid);

  for(PointT p : cloud->points)
  {
    PointT q;
    q.x = p.x - centroid.x;
    q.y = p.y - centroid.y;
    q.z = p.z - centroid.z;
    out_cloud->points.push_back(q);
  }

  return out_cloud;
}

void normalizePointCloud(CloudT::Ptr &out_cloud,
  CloudT::Ptr cloud ,
  double max_val = 1.0)
{
  PointT p_max;
  PointT p_min;
  pcl::getMinMax3D(*cloud, p_min, p_max);
  for(PointT p : cloud->points)
  {
    // ! Check for NaN points
    if(pcl::isFinite(p))
    {
      PointT q;
      q.x = (max_val)*(p.x - p_min.x)/(p_max.x - p_min.x);
      q.y = (max_val)*(p.y - p_min.y)/(p_max.y - p_min.y);
      q.z = (max_val)*(p.z - p_min.z)/(p_max.z - p_min.z);
      out_cloud->points.push_back(q);
    }
  }
  return ;
}

void preprocessPointCloud(
  pcl::PointCloud<pcl::PointXYZ>::Ptr &out_cloud,
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud
)
{
  normalizePointCloud(out_cloud, cloud);
  out_cloud = transposeToMassCenter(cloud);
  return ;
}

int main(int argc, char *argv[])
{
  // > Variables
  CloudT::Ptr cloud(new CloudT);
  CloudT::Ptr out_cloud(new CloudT);
  std::vector<cg::vertex> vertices;
  std::vector<cg::facade> facades;

  // > Parameters
  std::string file_path = "";
  double step_size = DEFAULT_STEP_SIZE;
  bool visualize = false;
  bool reduntant = false;

  // > Flow Control
  bool error_params = false;

  if(argc < 2)
  {
    std::cout << "> Error: Arguments expected." << std::endl;
    std::cout << "> Possible arguments:" << std::endl;
    std::cout << "'-p <FILE PATH>'" << std::endl;
    std::cout << "'-v' - Use this to visualize results" << std::endl;
    std::cout << "'-s <STEP SIZE>' (default is " <<
        DEFAULT_STEP_SIZE << ")" << std::endl;
    return -1;
  }

  // > Parameters reading
  int count = 1;

  while(count < argc)
  {
    // > Argument type
    std::string param = std::string(argv[count++]);
    // > File path
    if(!param.compare("-p"))
      file_path = std::string(argv[count++]);
    // > Step size parameter
    else if(!param.compare("-s"))
      step_size = atof(argv[count++]);
    else if(!param.compare("-v"))
      visualize = true;
    else if(!param.compare("-r"))
      redundant = true;
    else
    {
      std::cout << "=> Warning: argument " << param << " is not a default argument" << std::endl;
      count++;
    }
  }

  if(file_path.compare("") == 0)
  {
    std::cout << "> Argument missing: -p <FILE PATH>." << std::endl;
    error_params = true;
  }

  if(step_size <= 0 || step_size >= 1)
  {
    std::cout << "> Invalid argument: <STEP SIZE> must be greater than zero " <<
        "and smaller than one." << std::endl;
    error_params = true;
  }

  if(error_params)
  {
    std::cout << "> Detected errors. Program stopped." << std::endl;
    return -1;
  }

  if(utils::loadOFFFile(file_path, vertices, facades) != 0)
  {
    std::cout << "Error loading the file." << std::endl;
    return -1;
  }

  cg::Mesh mesh(vertices, facades);
  mesh.set_redundancy(true);
  mesh.normalize();
  mesh.centralize();
  mesh.to_cloud(cloud, step_size);

  //preprocessPointCloud(out_cloud, cloud);

  if(visualize)
  {
    pcl::visualization::CloudViewer viewer("OFF2PCD");
    viewer.showCloud (cloud);

    while (!viewer.wasStopped ()){}
  }

  return 0;
}
