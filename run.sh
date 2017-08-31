#!/bin/bash

# PARAMETERS
#######################

# > Absolute or relative path to the input file
#FILE_PATH="/path/to/file.off"
FILE_PATH="/media/braile/HDD/Workspace/Datasets/ModelNet10/sofa/train/sofa_0001.off"
# > Absolute or relative path to the output file
#OUTPUT_PATH="/path/to/output/file.pcd"
OUTPUT_PATH=".output/out.pcd"
# > The step size for the rastering algorithm (between 0 and 1)
STEP_SIZE="0.1"
# > Enable visualization. Comment this line to disable visualization.
VISUALIZE="-v"
# > Enable points redundancy. Comment this line to disable redundancy.
REDUNDANCY="-r"

#######################

mkdir .build
cd .build
cmake ..
make
cd ..
.build/main -p ${FILE_PATH} -o ${OUTPUT_PATH} -s ${STEP_SIZE} ${VISUALIZE} ${REDUNDANCY}
