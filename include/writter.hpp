#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>

namespace utils
{
  void write_pcd(
    std::string path,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud
  )
  {
    ofstream out_file;
    out_file.open(path);
    out_file << "# .PCD v.7 - Point Cloud Data file format" << std::endl;
    out_file << "VERSION .7" << std::endl;
    out_file << "FIELDS x y z" << std::endl;
    out_file << "SIZE 4 4 4" << std::endl;
    out_file << "TYPE F F F" << std::endl;
    out_file << "COUNT 1 1 1" << std::endl;
    out_file << "WIDTH " << cloud->size() << std::endl;
    out_file << "HEIGHT 1" << std::endl;
    out_file << "VIEWPOINT 0 0 0 1 0 0 0" << std::endl;
    out_file << "POINTS " << cloud->size() << std::endl;
    out_file << "DATA ascii" << std::endl;
    for(pcl::PointXYZ p : cloud->points)
      out_file << p.x << " " << p.y << " " << p.z << std::endl;
    return ;
  }
}
