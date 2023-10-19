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
#include <pcl/filters/voxel_grid.h>
// Usage 
// ./build_map [datset folder]  [outputfile name]

int build_map(std::string, std::string);
int accumulate_pcd_per_folder(std::string , pcl::PointCloud<pcl::PointXYZ>::Ptr ); 
std::vector<std::string> get_directories(std::string& );

int main (int argc, char** argv)

{
    std::string dataset_folder = std::string(argv[1]);
    std::string outputfile = argv[2];

    build_map(dataset_folder,outputfile);

    return (0);
}



int build_map(std::string folderpath, std::string outputfile)
{

    std::vector<std::string> dirs = get_directories(folderpath);
    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_pcd (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_accumulated_pcd (new pcl::PointCloud<pcl::PointXYZ>);


    for (auto i: dirs)
    {
        std::cout << i << std::endl;
        accumulate_pcd_per_folder(i, accumulated_pcd);
    }

    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (accumulated_pcd);
    sor.setLeafSize (2.0f, 2.0f, 2.0f);
    sor.filter (*downsampled_accumulated_pcd);

    pcl::io::savePCDFile (folderpath + "/" +outputfile + ".pcd", *accumulated_pcd,  true);
    pcl::io::savePCDFile (folderpath + "/" +outputfile + "_2.pcd", *downsampled_accumulated_pcd,  true);

    return(0);
}

int accumulate_pcd_per_folder(std::string folderpath, pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_pcd) 
{

    for (const auto & entry : std::filesystem::directory_iterator(folderpath))
    {
        std::cout << entry.path() << std::endl;
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ> (entry.path(), *cloud) == -1) //* load the file
        {
            PCL_ERROR ("Couldn't read file  \n");
            return (-1);
        }
        *accumulated_pcd += *cloud;
    }
}


std::vector<std::string> get_directories(std::string& s)
{
    std::vector<std::string> r;
    for(auto& p : std::filesystem::directory_iterator(s))
        if (p.is_directory())
            r.push_back(p.path().string());
    return r;
}