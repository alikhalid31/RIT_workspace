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


// Usage 
// ./map_compression [datset folder] [day1 (reference day)] [day2 (day to be compressed)] [resolution of octree] [output folder]

int find_change (std::string, std::string, std::string, std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr , float, float , std::vector<int>&, std::vector<float>&, std::vector<float>&, std::vector<float>&);
std::vector<std::string> splitString(std::string , char );
int compress_frames (std::string , std::string, float, std::string);


int main (int argc, char** argv)

{
    std::string dataset_folder = std::string(argv[1]);
    std::string reference_day_path = dataset_folder + "/" +std::string(argv[2]);
    std::string compress_day_path = dataset_folder + "/" + std::string(argv[3]);
    float resolution = atof(argv[4]);
    std::string output_folder = std::string(argv[5]);



    compress_frames(compress_day_path, reference_day_path, resolution, output_folder);

    return (0);
}


int compress_frames (std::string compress_day_path, std::string reference_day_path,  float resolution, std::string output_folder)
{
    std::string line ;
    std::ifstream association_file(compress_day_path+"/association.txt");
    std::vector<float> run_time_1, run_time_2, run_time_3, run_time_4;
    std::vector<float> run_time_total_wload, run_time_total_woload;
    float avg_wload, avg_woload;
    //float k = CLOCKS_PER_SEC;



    pcl::PointCloud<pcl::PointXYZ>::Ptr change_add (new pcl::PointCloud<pcl::PointXYZ> );
    //pcl::PointCloud<pcl::PointXYZI>::Ptr change_remove (new pcl::PointCloud<pcl::PointXYZI> );
    
    while (getline (association_file, line )) 
    {
        std::vector<std::string> split_line = splitString(line, ',');
        std::string compress_frame = split_line[0];
        std::string reference_frame = split_line[1];
        float distance = std::stof(split_line[2]);
        std::vector<int> indices_to_remove;
        std::cout <<compress_frame<<std::endl;

        if (find_change (reference_day_path, reference_frame, compress_day_path, compress_frame, change_add, distance, resolution,indices_to_remove,run_time_1, run_time_2, run_time_3 ) == 0 )
        {
            pcl::io::savePCDFileASCII (compress_day_path + "/" + output_folder + "/add/"+ compress_frame +".pcd", *change_add);
            //pcl::io::savePCDFileASCII (compress_day_path + "/" + output_folder + "/remove/"+ compress_frame +".pcd", *change_remove);
            std::ofstream outfile(compress_day_path + "/" + output_folder + "/remove_indices/"+ compress_frame +".dat", std::ios::out | std::ofstream::binary);
            std::ostream_iterator<int> out_itr(outfile, "\n"); 
            std::copy(indices_to_remove.begin(), indices_to_remove.end(),out_itr );
            outfile.close();
            run_time_4.push_back(clock());

            // std::ofstream outfile1(compress_day_path + "/" + output_folder + "/remove_indices_b/"+ compress_frame +".bin", std::ofstream::binary);
            // outfile1.write(reinterpret_cast<const char*>(indices_to_remove.data() /* or &v[0] pre-C++11 */), sizeof(int) * indices_to_remove.size());
            // outfile1.close();

        }

    }

    for(std::vector<float>::size_type i = 0; i != run_time_1.size(); i++) 
    {
        run_time_total_wload.push_back((float)(run_time_4[i]-run_time_1[i])/CLOCKS_PER_SEC);
        run_time_total_woload.push_back((float)(run_time_3[i]-run_time_2[i])/CLOCKS_PER_SEC);
    }
    //std::transform (run_time_1.begin(), run_time_1.end(), run_time_4.begin(), run_time_total_wload.begin(), std::minus<float>());
    //std::transform (run_time_total_wload.begin(), run_time_total_wload.end(), run_time_total_wload.begin(),  [k](float &c){ return c/k; });
    avg_wload= std::reduce (run_time_total_wload.begin(), run_time_total_wload.end()) / run_time_total_wload.size();
    

    //std::transform (run_time_2.begin(), run_time_2.end(), run_time_3.begin(), run_time_total_woload.begin(), std::minus<float>());
    //std::transform (run_time_total_woload.begin(), run_time_total_woload.end(), run_time_total_woload.begin(),  [k](float &c){ return c/k; });
    avg_woload= std::reduce (run_time_total_woload.begin(), run_time_total_woload.end()) / run_time_total_woload.size();

    std::cout << output_folder << std::endl;
    std::cout <<"Average_time with load: " << avg_wload<< std::endl;
    std::cout <<"Average_time without load: " << avg_woload<< std::endl;


    association_file.close();
    return (0);
}

int find_change (std::string reference_day_path, std::string reference_frame, std::string compress_day_path,  std::string compress_frame, pcl::PointCloud<pcl::PointXYZ>::Ptr change_add, 
float distance, float resolution,  std::vector<int>& indices_to_remove, std::vector<float>& run_time_1, std::vector<float>& run_time_2, std::vector<float>& run_time_3)
{
    run_time_1.push_back(clock());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_reference (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_compress (new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_reference_I (new pcl::PointCloud<pcl::PointXYZI>);
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_compress_I (new pcl::PointCloud<pcl::PointXYZI>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> (compress_day_path + "/pcds/" + compress_frame + ".pcd", *cloud_compress) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file day 1  \n");
        return (-1);
    }
    

    if (distance < 2.0)
    {
        if (pcl::io::loadPCDFile<pcl::PointXYZ> (reference_day_path + "/pcds/" + reference_frame + ".pcd", *cloud_reference) == -1) //* load the file
        {
            PCL_ERROR ("Couldn't read file day 1  \n");
            return (-1);
        }

        // if (pcl::io::loadPCDFile<pcl::PointXYZI> (reference_day_path + "/pcds/" + reference_frame + ".pcd", *cloud_reference_I) == -1) //* load the file
        // {
        //     PCL_ERROR ("Couldn't read file day 1  \n");
        //     return (-1);
        // }

        

        // if (pcl::io::loadPCDFile<pcl::PointXYZI> (compress_day_path + "/pcds/" + compress_frame + ".pcd", *cloud_compress_I) == -1) //* load the file
        // {
        //     PCL_ERROR ("Couldn't read file day 1  \n");
        //     return (-1);
        // }
    
        run_time_2.push_back(clock());

        pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
        pcl::ExtractIndices<pcl::PointXYZ> extract;

        pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree_change_add (resolution);
        octree_change_add.setInputCloud (cloud_reference);
        octree_change_add.addPointsFromInputCloud ();
        octree_change_add.switchBuffers ();
        octree_change_add.setInputCloud (cloud_compress);
        octree_change_add.addPointsFromInputCloud ();
        std::vector<int> newPointIdxVector_add;
        octree_change_add.getPointIndicesFromNewVoxels (newPointIdxVector_add);
        inliers->indices = newPointIdxVector_add;
        extract.setInputCloud (cloud_compress);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*change_add);

        pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree_change_remove (resolution);
        octree_change_remove.setInputCloud (cloud_compress);
        octree_change_remove.addPointsFromInputCloud ();
        octree_change_remove.switchBuffers ();
        octree_change_remove.setInputCloud (cloud_reference);
        octree_change_remove.addPointsFromInputCloud ();
        //std::vector<int> newPointIdxVector_remove;
        octree_change_remove.getPointIndicesFromNewVoxels (indices_to_remove);

        //indices_to_remove = newPointIdxVector_remove;

        run_time_3.push_back(clock());
        // inliers->indices = newPointIdxVector_remove;
        // extract.setInputCloud (cloud_reference_I);
        // extract.setIndices (inliers);
        // extract.setNegative (false);
        // extract.filter (*change_remove);
    }
    else
    {
        *change_add = *cloud_compress;
    }

    return(0);
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

