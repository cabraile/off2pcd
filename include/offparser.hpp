#ifndef __GUARD_OFF_PARSER__
#define __GUARD_OFF_PARSER__

#include <iostream>
#include <fstream>
#include "cg.hpp"

namespace utils
{
  // Parser
  // url: https://bytes.com/topic/c/answers/133131-reading-file-off-format
  // Thanks to Alexander Schmidt (access on 25 august 2017)
  int loadOFFFile(std::string filename,
    std::vector<cg::vertex> & vertices,
    std::vector<cg::facade> & facades)
  {
    int nv, nf;
    // Container holding last line read
    std::string readLine;
    // Containers for delimiter positions
    int delimiterPos_1, delimiterPos_2, delimiterPos_3, delimiterPos_4;

    // Open file for reading
    std::ifstream in(filename.c_str());

    // Check if file is in OFF format
    std::getline(in,readLine);
    if (readLine.compare("OFF") != 0)
      return -1;

    // Read values for Nv and Nf
    std::getline(in,readLine);
    delimiterPos_1 = readLine.find(" ", 0);
    nv = atoi(readLine.substr(0,delimiterPos_1+1).c_str());
    delimiterPos_2 = readLine.find(" ", delimiterPos_1);
    nf = atoi(readLine.substr(delimiterPos_1,delimiterPos_2 +1).c_str());

    // Read the vertices
    vertices.resize(nv);

    for (int n=0; n<nv; n++)
    {
      std::getline(in,readLine);
      delimiterPos_1 = readLine.find(" ", 0);
      vertices[n].x = atof(readLine.substr(0,delimiterPos_1).c_str());
      delimiterPos_2 = readLine.find(" ", delimiterPos_1+1);
      vertices[n].y =
          atof(readLine.substr(delimiterPos_1,delimiterPos_2 ).c_str());
      delimiterPos_3 = readLine.find(" ", delimiterPos_2+1);
      vertices[n].z =
          atof(readLine.substr(delimiterPos_2,delimiterPos_3 ).c_str());
    }

    // Read the facades
    facades.resize(nf);

    for (int n=0; n<nf; n++)
    {
      std::getline(in,readLine);
      delimiterPos_1 = readLine.find(" ", 0);
      delimiterPos_2 = readLine.find(" ", delimiterPos_1+1);
      facades[n].v1 =
          atoi(readLine.substr(delimiterPos_1,delimiterPos_2 ).c_str());
      delimiterPos_3 = readLine.find(" ", delimiterPos_2+1);
      facades[n].v2 =
          atoi(readLine.substr(delimiterPos_2,delimiterPos_3 ).c_str());
      delimiterPos_4 = readLine.find(" ", delimiterPos_3+1);
      facades[n].v3 =
          atoi(readLine.substr(delimiterPos_3,delimiterPos_4 ).c_str());

      //std::cout << facades[n].v1 << "\t" << facades[n].v2 << "\t" << facades[n].v3 << "\t" << std::endl;
    }

    return 0;
  }

  int loadXYZFile(std::string filename,
    pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud
  )
  {
    std::string readLine="";
    // Open file for reading
    std::ifstream in_file(filename);
    if(!in_file.is_open())
      return -1;
    while(std::getline(in_file,readLine))
    {
      pcl::PointXYZ p;
      int delimiterPos_1 = readLine.find(" ", 0);
      p.x = atof(readLine.substr(0,delimiterPos_1).c_str());
      int delimiterPos_2 = readLine.find(" ", delimiterPos_1+1);
      p.y = atof(readLine.substr(delimiterPos_1,delimiterPos_2 ).c_str());
      int delimiterPos_3 = readLine.find(" ", delimiterPos_2+1);
      p.z = atof(readLine.substr(delimiterPos_2,delimiterPos_3 ).c_str());
      cloud->points.push_back(p);
    }
    in_file.close();
    return 0;
  }

}

#endif
