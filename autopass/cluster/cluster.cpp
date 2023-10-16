
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
#include <pcl/segmentation/extract_clusters.h>



// Usage 
// ./DIFF1 [datset folder] [infra path]  [output folder]   [resolution ] 
// ./DIFF1 ~/Downloads/autopass_diff/data_3 infra/pcds infra/octree_0.1 0.1 

// int find_change (std::string,   pcl::PointCloud<pcl::PointXYZ>::Ptr, float, std::vector<float>&, std::vector<float>&, std::vector<float>&, std::vector<float>&, std::vector<float>&, std::vector<float>&, 
// std::string,  pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ>&,pcl::io::OctreePointCloudCompression<pcl::PointXYZ>*);
int clusters_frames (std::string pcd_folder,  std::string output_folder, int frame);
// std::vector<std::string> get_pose(std::string , std::string );
// Eigen::Matrix4f convert_YPR_to_rotation_matrix(float, float, float);
// Eigen::Matrix4f get_transformation_matrix(std::vector<std::string> );

int main (int argc, char** argv)

{
    std::string dataset_folder = std::string(argv[1]);
    std::string pcd_folder = dataset_folder + "/" + std::string(argv[2]);
    std::string output_folder = dataset_folder + "/" +  std::string(argv[3]);
    int frame = atoi(argv[4]);


    clusters_frames(pcd_folder , output_folder, frame);

    return (0);
}

int clusters_frames (std::string pcd_folder,  std::string output_folder, int frame)
{

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
    std::copy(std::filesystem::directory_iterator(pcd_folder), std::filesystem::directory_iterator(), std::back_inserter(files_in_directory));
    std::sort(files_in_directory.begin(), files_in_directory.end(), customLess);


    for (const std::string& filename : files_in_directory) {
        std::cout << filename << std::endl; // printed in alphabetical order
        std::string current_frame = filename;
        std::string current_frame_name = current_frame.substr(current_frame.find_last_of("/\\") + 1);
        std::cout << current_frame_name << std::endl;

        std::string output_folder_1 = output_folder +"/" +current_frame_name;


        if (!std::filesystem::is_directory(output_folder_1) || !std::filesystem::exists(output_folder_1)) 
        { 
            std::filesystem::create_directories(output_folder_1);
        }
        //std::string current_frame = pcd_folder +"/"+std::to_string(frame)+".pcd";

        pcl::PointCloud<pcl::PointXYZ>::Ptr pcd (new pcl::PointCloud<pcl::PointXYZ> );

        if (pcl::io::loadPCDFile<pcl::PointXYZ> (current_frame, *pcd) == -1) //* load the file
        {   
            PCL_ERROR ("Couldn't read file 1 \n");
            return (-1);
        }
        int clusters = 0;


        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud (pcd);
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance (0.9); // 2cm
        ec.setMinClusterSize (100);
        ec.setMaxClusterSize (250000);
        ec.setSearchMethod (tree);
        ec.setInputCloud (pcd);
        ec.extract (cluster_indices);

        std::cout << cluster_indices.size() << std::endl;

        pcl::PCDWriter writer;
        int j = 0;
        for (const auto& cluster : cluster_indices)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
            for (const auto& idx : cluster.indices) {
            cloud_cluster->push_back((*pcd)[idx]);
            } //*
            cloud_cluster->width = cloud_cluster->size ();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            //std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
            std::stringstream ss;
            ss << std::setw(4) << std::setfill('0') << j;
            writer.write<pcl::PointXYZ> (output_folder_1 + "/cloud_cluster_" + ss.str () + ".pcd", *cloud_cluster, false); //*
            j++;
        }


    }


    return (0);
}

// int find_change (std::string current_frame, pcl::PointCloud<pcl::PointXYZ>::Ptr previous_pcd, float resolution, std::vector<float>& run_time_1, std::vector<float>& run_time_2, 
// std::vector<float>& run_time_3,std::vector<float>& run_time_4, std::vector<float>& run_time_5,std::vector<float>& run_time_6, 
// std::string output_folder, pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ>& octree_change_add,
// pcl::io::OctreePointCloudCompression<pcl::PointXYZ>* octree_compression)

// {
//     std::cout << current_frame <<std::endl;
//     std::stringstream compressedData;

//     pcl::PointCloud<pcl::PointXYZ>::Ptr current_pcd (new pcl::PointCloud<pcl::PointXYZ> );
//     //pcl::PointCloud<pcl::PointXYZ>::Ptr previous_pcd (new pcl::PointCloud<pcl::PointXYZ> );

//     pcl::PointCloud<pcl::PointXYZ>::Ptr current_result (new pcl::PointCloud<pcl::PointXYZ> );
//     //pcl::PointCloud<pcl::PointXYZ>::Ptr previous_result (new pcl::PointCloud<pcl::PointXYZ> );
//     pcl::PointCloud<pcl::PointXYZ>::Ptr reonstructed_frame (new pcl::PointCloud<pcl::PointXYZ> );
//     pcl::PointCloud<pcl::PointXYZ>::Ptr reonstructed_frame_1 (new pcl::PointCloud<pcl::PointXYZ> );

//     pcl::PointCloud<pcl::PointXYZ>::Ptr compressed (new pcl::PointCloud<pcl::PointXYZ> );






//     float resolution2 = resolution*resolution;

//     if (pcl::io::loadPCDFile<pcl::PointXYZ> (current_frame, *current_pcd) == -1) //* load the file
//     {   
//         PCL_ERROR ("Couldn't read file 1 \n");
//         return (-1);
//     }

  

//     std::string current_frame_name = current_frame.substr(current_frame.find_last_of("/\\") + 1);
 
//     pcl::ExtractIndices<pcl::PointXYZ> extract;
//     pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());


//     run_time_1.push_back(clock());
//     std::vector<int> newPointIdxVector_add;
    
//     octree_change_add.setInputCloud (current_pcd);
        
//     run_time_2.push_back(clock());

//     octree_change_add.addPointsFromInputCloud ();
//     octree_change_add.getPointIndicesFromNewVoxels (newPointIdxVector_add);
//     octree_change_add.deleteCurrentBuffer();

//     inliers->indices = newPointIdxVector_add;
//     extract.setInputCloud (current_pcd);
//     extract.setIndices (inliers);
//     extract.setNegative (false);
//     extract.filter (*current_result);
    
//     run_time_3.push_back(clock());
//     pcl::io::OctreePointCloudCompression<pcl::PointXYZ> octree_compression1 (pcl::io::MANUAL_CONFIGURATION,false, 0.001, 0.1, false, 1, false, 8);


//     octree_compression1.encodePointCloud(current_result, compressedData);

//     run_time_4.push_back(clock());

//     octree_compression1.decodePointCloud(compressedData, compressed);

//     run_time_5.push_back(clock());

//     *reonstructed_frame = *previous_pcd + *current_result;
    
//     run_time_6.push_back(clock());

//     *reonstructed_frame_1 = *previous_pcd + *compressed;

//     octree_compression1.encodePointCloud(current_result, compressedData);

//     std::ofstream outFile(output_folder + "/compressed_change/"+ current_frame_name+".txt");
//     outFile << compressedData.rdbuf();
//     outFile.close();

//     pcl::io::savePCDFileASCII (output_folder+"/decompressed_change/"+ current_frame_name , *compressed);
//     pcl::io::savePCDFileASCII (output_folder+"/reconstructed_compressed/"+ current_frame_name , *reonstructed_frame_1);

//     pcl::io::savePCDFileASCII (output_folder+"/change/"+ current_frame_name , *current_result);
//     pcl::io::savePCDFileASCII (output_folder+"/reconstructed/"+ current_frame_name , *reonstructed_frame);

    

    

//     return(0);
// }

// std::vector<std::string> splitString(std::string str, char splitter){
//     std::vector<std::string> result;
//     std::string current = ""; 
//     for(int i = 0; i < str.size(); i++){
//         if(str[i] == splitter){
//             if(current != ""){
//                 result.push_back(current);
//                 current = "";
//             } 
//             continue;
//         }
//         current += str[i];
//     }
//     if(current.size() != 0)
//         result.push_back(current);
//     return result;
// }

