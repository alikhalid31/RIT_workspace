#!/bin/bash
echo " running error calculation script"

# ./error_calculation ~/data/map_compression/data_2/ego/town02/dropoff=0.0/noise=0.0 day2 draco_decompression_qp0 1 0
# ./error_calculation ~/data/map_compression/data_2/ego/town02/dropoff=0.1/noise=0.0 day2 draco_decompression_qp0 1 0
# ./error_calculation ~/data/map_compression/data_2/ego/town02/dropoff=0.2/noise=0.0 day2 draco_decompression_qp0 1 0
# ./error_calculation ~/data/map_compression/data_2/ego/town02/dropoff=0.3/noise=0.0 day2 draco_decompression_qp0 1 0
# ./error_calculation ~/data/map_compression/data_2/ego/town02/dropoff=0.4/noise=0.0 day2 draco_decompression_qp0 1 0
# ./error_calculation ~/data/map_compression/data_2/ego/town02/dropoff=0.5/noise=0.0 day2 draco_decompression_qp0 1 0

# ./error_calculation ~/data/map_compression/data_2/ego/town02/dropoff=0.0/noise=0.1 day2 draco_decompression_qp0 1 0
# ./error_calculation ~/data/map_compression/data_2/ego/town02/dropoff=0.0/noise=0.2 day2 draco_decompression_qp0 1 0
# ./error_calculation ~/data/map_compression/data_2/ego/town02/dropoff=0.0/noise=0.3 day2 draco_decompression_qp0 1 0

#./error_calculation ~/data/map_compression/data_3/ego/town02/t=10 day2 decompression_4a2_2 1 1
#./error_calculation ~/data/map_compression/data_3/ego/town02/t=30 day2 decompression_4a2_2 1 1
#./error_calculation ~/data/map_compression/data_3/ego/town02/t=50 day2 decompression_4a2_2 1 1
#./error_calculation ~/data/map_compression/data_3/ego/town02/t=70 day2 decompression_4a2_2 1 1
#./error_calculation ~/data/map_compression/data_3/ego/town02/t=90 day2 decompression_4a2_2 1 1

#./error_calculation ~/data/map_compression/data_3/ego/town02/t=30 day2 draco_decompression_qp14_c10 1 0
#./error_calculation ~/data/map_compression/data_3/ego/town02/t=50 day2 draco_decompression_qp14_c10 1 0
#./error_calculation ~/data/map_compression/data_3/ego/town02/t=70 day2 draco_decompression_qp14_c10 1 0
#./error_calculation ~/data/map_compression/data_3/ego/town02/t=90 day2 draco_decompression_qp14_c10 1 0

#./error_calculation ~/data/map_compression/data_3/ego/town02/t=10 day2 draco_decompression_qp11_c10 1 0
#./error_calculation ~/data/map_compression/data_3/ego/town02/t=30 day2 draco_decompression_qp11_c10 1 0
#./error_calculation ~/data/map_compression/data_3/ego/town02/t=50 day2 draco_decompression_qp11_c10 1 0
#./error_calculation ~/data/map_compression/data_3/ego/town02/t=70 day2 draco_decompression_qp11_c10 1 0
#./error_calculation ~/data/map_compression/data_3/ego/town02/t=90 day2 draco_decompression_qp11_c10 1 0


#for i in {10..90..20}
#do
#    ./error_calculation ~/data/map_compression/data_4/ego/town02/t=$i day2 decompression_4a2_2 1 1

#done

#for i in {10..90..20}
#do
#    ./error_calculation ~/data/map_compression/data_4/ego/town02/t=$i day2 decompression_4b 1 1

#done

#for i in {10..90..20}
#do
#./error_calculation ~/data/map_compression/data_4/ego/town02/t=$i day2 draco_decompression_qp14_c10  1 0

#done



# for i in {1..9}
# do
#     ./error_calculation ~/data/map_compression/data_4/ego/town02/t=70 day2 decompression_4a2_2_0.$i 1 1

# done


#for i in {1..9}
#do
#    ./error_calculation ~/data/map_compression/data_4/ego/town02/t=70 day2 decompression_4b_0.$i 1 1

#done

# for i in {0..5}
# do
# ./error_calculation ./../DIFF/data infra kdtree1_0.$i/reconstructed 1 1
# done

for i in {1..9}
do 
./error_calculation ~/Downloads/autopass_diff/data_3 infra infra/octree_0.$i/reconstructed 1 1
done 

# for i in {1..6}
# do
#     for j in {1..9}
#     do
#     echo "infra_subset_$i octree resolution $j"
#     ./error_calculation ~/Downloads/autopass_diff/data_1 infra_subset_$i  infra_subset_$i/octree_0.$j/reconstructed 1 1
#     done
# done

# for i in {1..5}
# do
# ./error_calculation ./../DIFF1/data infra octree2_0.$i/reconstructed 1 1
# done



# ./error_calculation ~/data/map_compression/data_3/ego/town02/t=30 day2 decompression_4a2_2 1 1
# ./error_calculation ~/data/map_compression/data_3/ego/town02/t=50 day2 decompression_4a2_2 1 1
# ./error_calculation ~/data/map_compression/data_3/ego/town02/t=70 day2 decompression_4a2_2 1 1
# ./error_calculation ~/data/map_compression/data_3/ego/town02/t=90 day2 decompression_4a2_2 1 1


# ./error_calculation ~/data/map_compression/data_2/ego/town02/dropoff=0.1/noise=0.0 day2 draco_decompression_qp14_c10  1 0
# ./error_calculation ~/data/map_compression/data_2/ego/town02/dropoff=0.2/noise=0.0 day2 draco_decompression_qp14_c10  1 0
# ./error_calculation ~/data/map_compression/data_2/ego/town02/dropoff=0.3/noise=0.0 day2 draco_decompression_qp14_c10  1 0
# ./error_calculation ~/data/map_compression/data_2/ego/town02/dropoff=0.4/noise=0.0 day2 draco_decompression_qp14_c10  1 0
# ./error_calculation ~/data/map_compression/data_2/ego/town02/dropoff=0.5/noise=0.0 day2 draco_decompression_qp14_c10  1 0

# ./error_calculation ~/data/map_compression/data_2/ego/town02/dropoff=0.0/noise=0.1 day2 draco_decompression_qp14_c10  1 0
# ./error_calculation ~/data/map_compression/data_2/ego/town02/dropoff=0.0/noise=0.2 day2 draco_decompression_qp14_c10  1 0
# ./error_calculation ~/data/map_compression/data_2/ego/town02/dropoff=0.0/noise=0.3 day2 draco_decompression_qp14_c10  1 0

# ./error_calculation ~/data/map_compression/data_2/ego/town02/dropoff=0.0/noise=0.0 day2 draco_decompression_10 1 0
# ./error_calculation ~/data/map_compression/data_2/ego/town02/dropoff=0.1/noise=0.0 day2 draco_decompression_10 1 0
# ./error_calculation ~/data/map_compression/data_2/ego/town02/dropoff=0.2/noise=0.0 day2 draco_decompression_10 1 0
# ./error_calculation ~/data/map_compression/data_2/ego/town02/dropoff=0.3/noise=0.0 day2 draco_decompression_10 1 0
# ./error_calculation ~/data/map_compression/data_2/ego/town02/dropoff=0.4/noise=0.0 day2 draco_decompression_10 1 0
# ./error_calculation ~/data/map_compression/data_2/ego/town02/dropoff=0.5/noise=0.0 day2 draco_decompression_10 1 0

# ./error_calculation ~/data/map_compression/data_2/ego/town02/dropoff=0.0/noise=0.1 day2 draco_decompression_10 1 0
# ./error_calculation ~/data/map_compression/data_2/ego/town02/dropoff=0.0/noise=0.2 day2 draco_decompression_10 1 0
# ./error_calculation ~/data/map_compression/data_2/ego/town02/dropoff=0.0/noise=0.3 day2 draco_decompression_10 1 0


#############################
#./error_calculation ../../data/data_folder/ego/town02/wo_noise/vehicles=12 day2 decompression_4b 1 1
#./error_calculation ../../data/data_folder/ego/town02/wo_noise/vehicles=6 day2 decompression_4b 1 1
#./error_calculation ../../data/data_folder/ego/town02/w_noise/vehicles=12 day2 decompression_4b 1 1
#./error_calculation ../../data/data_folder/ego/town02/w_noise/vehicles=6 day2 decompression_4b 1 1




