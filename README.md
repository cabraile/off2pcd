# OFF2PCD

## About

  The OFF2PCD converts `off` (aka object file format) - files to `pcd` (point cloud) files. 
  More information of each file format can be found at 
  [the Princeton website](http://segeval.cs.princeton.edu/public/off_format.html) 
  and 
  [the pointcloud documentation](http://pointclouds.org/documentation/tutorials/pcd_file_format.php).

## Usage

  Simply run `bash run.sh` from this project root folder. The parameters can be changed inside the `run.sh` script. They are:
  
  * *FILE_PATH*: Absolute or relative path to the input file
  * *OUTPUT_PATH*: Absolute or relative path to the output file
  * *STEP_SIZE*: The step size for the line fiiling algorithm (between 0 and 1)
  * *VISUALIZE*: Enable visualization. Comment the line to disable visualization.
  * *REDUNDANCY*: Enable points redundancy. Comment the line to disable redundancy.

