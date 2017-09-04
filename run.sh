#!/bin/bash

# PARAMETERS
#######################

# > Absolute or relative path to the input file
FILE_PATH="path/to/file.off"
# > Absolute or relative path to the output file
OUTPUT_PATH=".build/file.pcd"
# > The step size for the rastering algorithm (between 0 and 1)
STEP_SIZE="0.1"
# > Enable visualization. Comment the line to disable visualization.
VISUALIZE="-v"
# > Enable points redundancy. Comment the line to disable redundancy.
REDUNDANCY="-r"
# > Sample surface points only (don't add vertices and edges points).
# > Comment the line to compute all points.
FACES_ONLY="-f"

#######################

mkdir .build
cd .build
cmake ..
make
cd ..
.build/main -p ${FILE_PATH} -o ${OUTPUT_PATH} -s ${STEP_SIZE} ${VISUALIZE} ${REDUNDANCY} ${FACES_ONLY}
