#!/bin/bash

echo " running diff1 code "
cd DIFF1
./script.sh

echo " running compression code"
cd ..
cd 4b-map_compression_octree
./script.sh

echo " running erorr code"
cd ..
cd 6-error_calculation
./script.sh


#echo " running compression code "
#cd ..
#cd 4b-map_compression_octree
#./script.sh


#echo " running decompression code"
#cd ..
#cd 5b-map_decompression_octree
#./script.sh


#echo " running error calculation code"
#cd ..
#cd 6-error_calculation
#./script.sh

