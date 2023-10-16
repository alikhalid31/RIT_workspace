#1/bin/bash 

echo " running pcd to ply conversion code"

# for i in {10..90..20}
# do 
# 	input_folder="/home/ak5013/data/map_compression/data_4/ego/town02/t=$i/day2/pcds"
# 	output_folder="/home/ak5013/data/map_compression/data_4/ego/town02/t=$i/day2/plys"
# 	python pcd2ply.py --input_path $input_folder --output_path $output_folder

# done

input_folder="/home/ak5013/data/map_compression/data_1/ego/town02/wo_dropoff/wo_noise/vehicles=6/day2/pcds"
output_folder="/home/ak5013/data/map_compression/data_1/ego/town02/wo_dropoff/wo_noise/vehicles=6/day2/plys"
python pcd2ply.py --input_path $input_folder --output_path $output_folder

input_folder="/home/ak5013/data/map_compression/data_1/ego/town02/wo_dropoff/wo_noise/vehicles=12/day2/pcds"
output_folder="/home/ak5013/data/map_compression/data_1/ego/town02/wo_dropoff/wo_noise/vehicles=12/day2/plys"
python pcd2ply.py --input_path $input_folder --output_path $output_folder

#for i in {1..3}
#do 
#	input_folder="/home/ak5013/data/map_compression/data_2/ego/town02/dropoff=0.0/noise=0.$i/day2/pcds"
#	output_folder="/home/ak5013/data/map_compression/data_2/ego/town02/dropoff=0.0/noise=0.$i/day2/plys"

#	python pcd2ply.py --input_path $input_folder --output_path $output_folder


#done
