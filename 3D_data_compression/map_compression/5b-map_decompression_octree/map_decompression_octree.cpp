#include <iostream>
#include <fstream>
#include <filesystem>
#include <vector>  
#include <string.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/filters/extract_indices.h>
#include <ctime>


// Usage 
// ./map_compression [datset folder]  [day2 (day to be compressed)] [resolution of octree] [input folder] [output folder] [show-statistics (true or false)] [VoxelGridDownDownSampling (true or false)]
// ./map_decompression_octree ../data2 day2 0.8 octree_results/resolution=0.8/compression octree_results/resolution=0.8/decompression false false
std::vector<std::string> splitString(std::string , char );
int decompress_frames (std::string , float, std::string, std::string, bool, bool);
int decompress_frame (std::string, std::string, float,std::string, std::string, bool, bool, std::vector<float>&,  std::vector<float>&,  std::vector<float>&,  std::vector<float>&);

int main (int argc, char** argv)

{
    std::string dataset_folder = std::string(argv[1]);
    std::string compress_day_path = dataset_folder + "/" +std::string(argv[2]);
    float resolution = atof(argv[3]);
    std::string input_folder = std::string(argv[4]);
    std::string output_folder = std::string(argv[5]);
    std::string action1(argv[6]);
    std::string action2(argv[7]);
    

    bool showStatistics = false;
    if (action1 == "true") {
        showStatistics = true;
    }

    bool VoxelGridDownDownSampling = false;
    if (action2 == "true") {
        VoxelGridDownDownSampling = true;
    }

    decompress_frames(compress_day_path, resolution, input_folder, output_folder, showStatistics, VoxelGridDownDownSampling);

    return (0);
}

int decompress_frames(std::string compress_day_path, float resolution, std:: string input_folder, std::string output_folder, bool showStatistics, bool VoxelGridDownDownSampling)
{
    std::vector<float> run_time_1, run_time_2, run_time_3, run_time_4;
    std::vector<float> run_time_total_wload, run_time_total_woload;
    float avg_wload, avg_woload;

    if (!std::filesystem::is_directory(compress_day_path + "/"+output_folder) || !std::filesystem::exists(compress_day_path + "/"+output_folder)) 
        { 
            std::filesystem::create_directories(compress_day_path + "/"+output_folder);
        }


    for (const auto & entry : std::filesystem::directory_iterator(compress_day_path+ "/"+input_folder))
    {
        std::cout << entry.path() << std::endl;
        decompress_frame(compress_day_path, entry.path(), resolution, input_folder, output_folder, showStatistics, VoxelGridDownDownSampling,run_time_1, run_time_2, run_time_3, run_time_4);
    }

    for(std::vector<float>::size_type i = 0; i != run_time_1.size(); i++) 
    {
        run_time_total_wload.push_back((float)(run_time_4[i]-run_time_1[i])/CLOCKS_PER_SEC);
        run_time_total_woload.push_back((float)(run_time_3[i]-run_time_2[i])/CLOCKS_PER_SEC);
    }

    avg_wload= std::reduce (run_time_total_wload.begin(), run_time_total_wload.end()) / run_time_total_wload.size();
    avg_woload= std::reduce (run_time_total_woload.begin(), run_time_total_woload.end()) / run_time_total_woload.size();

    std::cout << output_folder << std::endl;
    std::cout <<"Average_time with load: " << avg_wload<< std::endl;
    std::cout <<"Average_time without load: " << avg_woload<< std::endl;
    return(0);
}

int decompress_frame (std::string compress_day_path, std::string frame_path,  float resolution, std::string input_folder, std::string output_folder, bool showStatistics, bool VoxelGridDownDownSampling,
std::vector<float>& run_time_1, std::vector<float>& run_time_2, std::vector<float>& run_time_3,  std::vector<float>& run_time_4)
{
    run_time_1.push_back(clock());

    pcl::PointCloud<pcl::PointXYZ>::Ptr compressed (new pcl::PointCloud<pcl::PointXYZ> );
    std::vector<std::string> split_1 = splitString(frame_path, '/');
    std::string frame_name = split_1.back();
    std::vector<std::string> frame_number = splitString(frame_name, '.');

    std::stringstream compressedData;
    std::ifstream inFile(compress_day_path + "/"+input_folder+ +"/"+frame_number[0]+"."+frame_number[1]+".txt");
    compressedData << inFile.rdbuf();
    inFile.close();

    run_time_2.push_back(clock());
    pcl::io::OctreePointCloudCompression<pcl::PointXYZ> octree_compression (pcl::io::MANUAL_CONFIGURATION,showStatistics, 0.001, resolution, VoxelGridDownDownSampling, 100, false, 8);
    octree_compression.decodePointCloud(compressedData, compressed);

    run_time_3.push_back(clock());

    pcl::io::savePCDFileASCII (compress_day_path + "/"+ output_folder +"/"+frame_number[0]+"."+frame_number[1] +".pcd", *compressed);

    run_time_4.push_back(clock());




    return (0);
}


std::vector<std::string> splitString(std::string str, char splitter){
    std::vector<std::string> result;
    std::string current = ""; 
    for(int i = 0; i < str.size(); i++){
        if(str[i] == splitter){
            if(current != ""){
                result.push_back(current);
                current = "";
            } 
            continue;
        }
        current += str[i];
    }
    if(current.size() != 0)
        result.push_back(current);
    return result;
}