#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/visualization/cloud_viewer.h>

#include "include/offparser.hpp"
#include "include/cg.hpp"
#include "include/writter.hpp"

#define DEFAULT_STEP_SIZE 0.1

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> CloudT;

int main(int argc, char *argv[])
{
  // > Variables
  CloudT::Ptr cloud(new CloudT);
  std::vector<cg::vertex> vertices;
  std::vector<cg::facade> facades;

  // > Parameters
  std::string file_path = "";
  std::string out_path = "";
  double step_size = DEFAULT_STEP_SIZE;
  bool visualize = false;
  bool redundant = false;

  // > Flow Control
  bool error_params = false;

  if(argc < 2)
  {
    std::cout << "> Error: Arguments expected." << std::endl;
    std::cout << "> Possible arguments:" << std::endl;
    std::cout << "'-p <FILE PATH>'" << std::endl;
    std::cout << "'-o <OUPUT PATH>'" << std::endl;
    std::cout << "'-v' - Use this to visualize results" << std::endl;
    std::cout << "'-r' - Use this to enable point redundancy" << std::endl;
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
    // > Output path
    else if(!param.compare("-o"))
      out_path = std::string(argv[count++]);
    // > Step size parameter
    else if(!param.compare("-s"))
      step_size = atof(argv[count++]);
    // > Enable visualization
    else if(!param.compare("-v"))
      visualize = true;
    // > Set redundancy
    else if(!param.compare("-r"))
      redundant = true;
    else
    {
      std::cout << "=> Warning: argument " << param <<
          " is not a default argument" << std::endl;
      count++;
    }
  }

  if(file_path.compare("") == 0)
  {
    std::cout << "> Argument missing: -p <FILE PATH>." << std::endl;
    error_params = true;
  }

  if(out_path.compare("") == 0)
  {
    std::cout << "> Argument missing: -o <OUTPUT PATH>." << std::endl;
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
  mesh.set_redundancy(redundant);
  mesh.normalize();
  mesh.to_cloud(cloud, step_size);

  utils::write_pcd(out_path, cloud);

  if(visualize)
  {
    pcl::visualization::CloudViewer viewer("OFF2PCD - Original");
    viewer.showCloud (cloud);

    while (!viewer.wasStopped ()){}
  }

  return 0;
}
