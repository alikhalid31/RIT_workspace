#include <iostream>
#include <string.h>
#include <fstream>
#include <vector> 
#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/filters/extract_indices.h>
#include <ctime>


// usage 
// ./map_decompression [dataset path] [reference day] [recover day] [resolution] [input folder] [output folder]

std::vector<std::string> splitString(std::string , char );
int recover_frame (std::string , std::string, float, std::string, std::string );


int main (int argc, char** argv)
{
    std::string data_path = std::string(argv[1]);
    std::string reference_day_path = data_path + "/" +std::string(argv[2]);
    std::string recover_day_path = data_path + "/" + std::string(argv[3]);
    float resolution = atof(argv[4]);
    std::string input_folder = std::string(argv[5]);
    std::string output_folder = std::string(argv[6]);
    recover_frame (recover_day_path, reference_day_path, resolution, input_folder, output_folder);

  return (0);
}

int recover_frame (std::string recover_day_path, std::string reference_day_path,  float resolution, std::string input_folder, std::string output_folder)
{
    std::ifstream association_file(recover_day_path+"/association.txt");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_add (new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_remove (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_recovered (new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_recovered_I (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    std::string line;

    std::vector<float> run_time_1, run_time_2, run_time_3, run_time_4;
    std::vector<float> run_time_total_wload, run_time_total_woload;
    float avg_wload, avg_woload;



    while (getline (association_file, line )) 
    {

        std::vector<std::string> split_line = splitString(line, ',');
        std::string recover_frame = split_line[0];
        std::string reference_frame = split_line[1];
        float distance = std::stof(split_line[2]);
      
        run_time_1.push_back(clock());

        if (distance < 2.0)
        {
            if (pcl::io::loadPCDFile<pcl::PointXYZ> (recover_day_path + "/" + input_folder +"/add/" + recover_frame + ".pcd" , *cloud_add) == -1) //* load the file
            {
                PCL_ERROR ("Couldn't read file day 1  \n");
                return (-1);
            }
            // if (pcl::io::loadPCDFile<pcl::PointXYZ> (recover_day_path + "/" + input_folder + "/remove/" + recover_frame + ".pcd", *cloud_remove) == -1) //* load the file
            // {
            //     PCL_ERROR ("Couldn't read file day 1  \n");
            //     return (-1);
            // }

            if (pcl::io::loadPCDFile<pcl::PointXYZ> (reference_day_path + "/pcds/" + reference_frame + ".pcd", *cloud_recovered) == -1) //* load the file
            {
                PCL_ERROR ("Couldn't read file day 1  \n");
                return (-1);
            }
            // if (pcl::io::loadPCDFile<pcl::PointXYZI> (reference_day_path + "/pcds/" + reference_frame + ".pcd", *cloud_recovered_I) == -1) //* load the file
            // {
            //     PCL_ERROR ("Couldn't read file day 1  \n");
            //     return (-1);
            // }

            // pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree_change_remove (resolution);
            // octree_change_remove.setInputCloud (cloud_recovered);
            // octree_change_remove.addPointsFromInputCloud ();
            // octree_change_remove.switchBuffers ();
            // octree_change_remove.setInputCloud (cloud_remove);
            // octree_change_remove.addPointsFromInputCloud ();
            std::vector<int> newPointIdxVector_remove;
            //octree_change_remove.getPointIndicesFromNewVoxels (newPointIdxVector_remove);

            std::ifstream fin(recover_day_path + "/" + input_folder + "/remove_indices/" + recover_frame + ".dat", std::fstream::binary | std::fstream::out);
            int num;
            while (fin >> num)
                newPointIdxVector_remove.push_back(num);
            // for (int i: newPointIdxVector_remove)
            //     std::cout << i << ' ';
            
            // std::cout << std::endl;
            run_time_2.push_back(clock());

            inliers->indices = newPointIdxVector_remove;
            extract.setInputCloud (cloud_recovered);
            extract.setIndices (inliers);
            extract.setNegative (true);
            extract.filter (*cloud_recovered);

            *cloud_recovered += *cloud_add;
            run_time_3.push_back(clock());


            pcl::io::savePCDFileASCII (recover_day_path + "/" + output_folder +"/"+ recover_frame +".pcd", *cloud_recovered);
            run_time_4.push_back(clock());

        }
        else
        {
            if (pcl::io::loadPCDFile<pcl::PointXYZ> (recover_day_path + "/pcds/" + recover_frame + ".pcd", *cloud_recovered) == -1) //* load the file
            {
                PCL_ERROR ("Couldn't read file day 1  \n");
                return (-1);
            }
            pcl::io::savePCDFileASCII (recover_day_path  + "/" + output_folder +"/"+ recover_frame +".pcd", *cloud_recovered);

        }


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

    association_file.close();


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