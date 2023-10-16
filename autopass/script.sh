#!/bin/bash

echo " running diff code "
cd diff
./diff /home/ak5013/Downloads/Yash_usecases/Scenario_1_red_light_violation/ Vehicle_World basemap.pcd vehicle_diff_0.9 0.9
./diff /home/ak5013/Downloads/Yash_usecases/Scenario_1_red_light_violation/ Fused_World basemap.pcd fused_diff_0.9 0.9
./diff /home/ak5013/Downloads/Yash_usecases/Scenario_2_unprotected_left_turn/ Vehicle_World basemap.pcd vehicle_diff_0.9 0.9
./diff /home/ak5013/Downloads/Yash_usecases/Scenario_2_unprotected_left_turn/ Fused_World basemap.pcd fused_diff_0.9 0.9


echo " running outlier code"
cd ../outlier
./outlier /home/ak5013/Downloads/Yash_usecases/Scenario_1_red_light_violation/ vehicle_diff_0.9 vehicle_outlier_0.9
./outlier /home/ak5013/Downloads/Yash_usecases/Scenario_1_red_light_violation/ fused_diff_0.9 fused_outlier_0.9
./outlier /home/ak5013/Downloads/Yash_usecases/Scenario_2_unprotected_left_turn/ vehicle_diff_0.9 vehicle_outlier_0.9
./outlier /home/ak5013/Downloads/Yash_usecases/Scenario_2_unprotected_left_turn/ fused_diff_0.9 fused_outlier_0.9



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

