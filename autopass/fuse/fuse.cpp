#include <iostream>
#include <fstream>
#include <filesystem>
#include <vector>  
#include <string.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/filters/extract_indices.h>
#include <ctime>
#include <pcl/compression/octree_pointcloud_compression.h>


// Usage 
// ./DIFF1 [datset folder] [infra path]  [output folder]   [resolution ] 
// ./DIFF1 ~/Downloads/autopass_diff/data_3 infra/pcds infra/octree_0.1 0.1 

int fuse_frames (std::string infra_path, std::string vehicle_path, std::string output_folder);

int main (int argc, char** argv)

{
    std::string dataset_folder = std::string(argv[1]);
    std::string infra_path = dataset_folder + "/" + std::string(argv[2]);
    std::string vehicle_path = dataset_folder + "/" + std::string(argv[3]);
    std::string output_folder = dataset_folder + "/" +  std::string(argv[4]);


    if (!std::filesystem::is_directory(output_folder) || !std::filesystem::exists(output_folder)) 
    { 
        std::filesystem::create_directories(output_folder);
    }
    fuse_frames(infra_path , vehicle_path, output_folder);

    return (0);
}


int fuse_frames (std::string infra_path, std::string vehicle_path, std::string output_folder)
{

    int count = 1; 

    // sorting the files before iterating on them 
    struct
    {
        bool operator()(std::filesystem::path a, std::filesystem::path b) const 
        {
            std::string a1 = a;
            std::string b1 = b;
            double af, bf;
            a1 = a1.substr(a1.find_last_of("/\\") + 1);
            b1 = b1.substr(b1.find_last_of("/\\") + 1);
            af = stof(a1.substr(0, a1.find_last_of(".\\") ));
            bf = stof(b1.substr(0, b1.find_last_of(".\\") ));
            //std::cout<<af<<std::endl;
            return af < bf; 
        }
    }
    customLess;


    std::vector<std::filesystem::path> files_in_directory;
    std::copy(std::filesystem::directory_iterator(infra_path), std::filesystem::directory_iterator(), std::back_inserter(files_in_directory));
    std::sort(files_in_directory.begin(), files_in_directory.end(), customLess);


    for (const std::string& filename : files_in_directory) {
        //std::cout << filename << std::endl; // printed in alphabetical order
        std::string current_frame = filename;
        std::string current_frame_name = current_frame.substr(current_frame.find_last_of("/\\") + 1);
        std::cout << current_frame_name << std::endl;
        std::string vehicle_pcd_path = vehicle_path + "/" +current_frame_name;

        pcl::PointCloud<pcl::PointXYZ>::Ptr infra_pcd (new pcl::PointCloud<pcl::PointXYZ> );
        pcl::PointCloud<pcl::PointXYZ>::Ptr vehicle_pcd (new pcl::PointCloud<pcl::PointXYZ> );
        pcl::PointCloud<pcl::PointXYZ>::Ptr fused_pcd (new pcl::PointCloud<pcl::PointXYZ> );

        if (pcl::io::loadPCDFile<pcl::PointXYZ> (current_frame, *infra_pcd) == -1) //* load the file
        {   
            PCL_ERROR ("Couldn't read file 1 \n");
            return (-1);
        }

        if (pcl::io::loadPCDFile<pcl::PointXYZ> (vehicle_pcd_path, *vehicle_pcd) == -1) //* load the file
        {   
            PCL_ERROR ("Couldn't read file 1 \n");
            return (-1);
        }

        *fused_pcd = *infra_pcd + *vehicle_pcd ;

        pcl::io::savePCDFileASCII (output_folder+"/"+ current_frame_name , *fused_pcd);

    }


    return (0);
}

