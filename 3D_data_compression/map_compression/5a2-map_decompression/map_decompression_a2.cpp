#include <iostream>
#include <string.h>
#include <fstream>
#include <filesystem>
#include <vector> 
#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/filters/extract_indices.h>
#include <ctime>


// usage 
// ./map_decompression [dataset path] [basemap_path] [reference day] [recover day] [resolution] [input folder] [output folder]
//./map_decompression_a2 ../../data/data_folder/ego/town02/w_noise/vehicles=12 ../../data/data_folder/basemaps/town_02/w_noise day1 day2 0.1 compression_4a2 decompression_4a2

std::vector<std::string> splitString(std::string , char );
int recover_frame (std::string , std::string, std::string,float, std::string, std::string );
std::vector<std::string> get_pose(std::string , std::string );
Eigen::Matrix4f convert_YPR_to_rotation_matrix(float, float, float);
Eigen::Matrix4f get_transformation_matrix(std::vector<std::string> );


int main (int argc, char** argv)
{
    std::string data_path = std::string(argv[1]);
    std::string basemap_path = std::string(argv[2]);
    std::string reference_day_path = data_path + "/" +std::string(argv[3]);
    std::string recover_day_path = data_path + "/" + std::string(argv[4]);
    float resolution = atof(argv[5]);
    std::string input_folder = std::string(argv[6]);
    std::string output_folder = std::string(argv[7]);

    recover_frame (recover_day_path, reference_day_path, basemap_path,resolution, input_folder, output_folder);

  return (0);
}

int recover_frame (std::string recover_day_path, std::string reference_day_path, std::string basemap_path,  float resolution, std::string input_folder, std::string output_folder)
{
    std::ifstream association_file(recover_day_path+"/association.txt");

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_add (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_recovered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base_recovered (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    std::string line;

    std::vector<float> run_time_1, run_time_2, run_time_3, run_time_4;
    std::vector<float> run_time_total_wload, run_time_total_woload;
    float avg_wload, avg_woload;

    pcl::PointCloud<pcl::PointXYZ>::Ptr base_cloud (new pcl::PointCloud<pcl::PointXYZ> );
    pcl::io::loadPCDFile<pcl::PointXYZ>(basemap_path+"/complete_pcl.pcd", *base_cloud);
    pcl::ExtractIndices<pcl::PointXYZ> extract_base;
    extract_base.setInputCloud (base_cloud);
    extract_base.setNegative (false);
    
    std::string output_path = recover_day_path + "/" + output_folder  ;


    if (!std::filesystem::is_directory(output_path) || !std::filesystem::exists(output_path)) 
        { 
            std::filesystem::create_directories(output_path);
        }



    while (getline (association_file, line )) 
    {

        std::vector<std::string> split_line = splitString(line, ',');
        std::string recover_frame = split_line[0];
        std::string reference_frame = split_line[1];
        float distance = std::stof(split_line[2]);

        //std::cout << recover_frame << std::endl;
      
        run_time_1.push_back(clock());

        if (distance < 2.0)
        {
            if (pcl::io::loadPCDFile<pcl::PointXYZ> (recover_day_path + "/" + input_folder +"/add/" + recover_frame + ".pcd" , *cloud_add) == -1) //* load the file
            {
                PCL_ERROR ("Couldn't read file day 1  \n");
                return (-1);
            }


            if (pcl::io::loadPCDFile<pcl::PointXYZ> (reference_day_path + "/pcds/" + reference_frame + ".pcd", *cloud_recovered) == -1) //* load the file
            {
                PCL_ERROR ("Couldn't read file day 1  \n");
                return (-1);
            }


            std::vector<int> indices_to_add,indices_to_remove ;

            std::ifstream fin(recover_day_path + "/" + input_folder + "/remove_indices/" + recover_frame + ".dat", std::fstream::binary | std::fstream::out);
            int num;
            while (fin >> num)
                indices_to_remove.push_back(num);

            std::ifstream fin1(recover_day_path + "/" + input_folder + "/add_indices/" + recover_frame + ".dat", std::fstream::binary | std::fstream::out);
            while (fin1 >> num)
                indices_to_add.push_back(num);


            std::vector<std::string> pose = get_pose(recover_day_path+"/pose.txt", recover_frame);
            Eigen::Matrix4f transformation_matrix = get_transformation_matrix(pose);

            run_time_2.push_back(clock());

            inliers->indices = indices_to_remove;
            extract.setInputCloud (cloud_recovered);
            extract.setIndices (inliers);
            extract.setNegative (true);
            extract.filter (*cloud_recovered);
            

            inliers->indices = indices_to_add;
            extract_base.setIndices (inliers);
            extract_base.filter (*cloud_base_recovered);
            pcl::transformPointCloud (*cloud_base_recovered, *cloud_base_recovered, transformation_matrix.inverse());


            *cloud_recovered += *cloud_add ;
            *cloud_recovered += *cloud_base_recovered ;
            run_time_3.push_back(clock());


            pcl::io::savePCDFileASCII (output_path +"/"+ recover_frame +".pcd", *cloud_recovered);
            run_time_4.push_back(clock());

        }
        else
        {
            if (pcl::io::loadPCDFile<pcl::PointXYZ> (recover_day_path + "/pcds/" + recover_frame + ".pcd", *cloud_recovered) == -1) //* load the file
            {
                PCL_ERROR ("Couldn't read file day 1  \n");
                return (-1);
            }
            pcl::io::savePCDFileASCII (output_path +"/"+ recover_frame +".pcd", *cloud_recovered);

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

Eigen::Matrix4f get_transformation_matrix(std::vector<std::string> pose )
{
    Eigen::Matrix4f rotaiton_matrix = convert_YPR_to_rotation_matrix(std::stof(pose[4]),std::stof(pose[5]),std::stof(pose[6]));
    rotaiton_matrix (0,3) = std::stof(pose[1]);
    rotaiton_matrix (1,3) = std::stof(pose[2]);
    rotaiton_matrix (2,3) = std::stof(pose[3]);

    return rotaiton_matrix;
}


Eigen::Matrix4f convert_YPR_to_rotation_matrix(float yaw, float pitch, float roll)
{
    Eigen::Matrix4f rotaiton_matrix = Eigen::Matrix4f::Identity();
    yaw = (M_PI *yaw)/180.0 ;
    pitch = (M_PI *pitch)/180.0 ;
    roll = (M_PI *roll)/180.0 ;
    rotaiton_matrix (0,0) = std::cos(yaw) * std::cos(pitch);
    rotaiton_matrix (1,0) = sin(yaw) * std::cos(pitch);
    rotaiton_matrix (2,0) = -sin(pitch);


    rotaiton_matrix (0,1) = std::cos(yaw)*sin(pitch)*sin(roll) - sin(yaw)*std::cos(roll);
    rotaiton_matrix (1,1) = sin(yaw)*sin(pitch)*sin(roll) + std::cos(yaw)*std::cos(roll);
    rotaiton_matrix (2,1) = std::cos(pitch)*sin(roll);

    rotaiton_matrix (0,2) = std::cos(yaw)*sin(pitch)*std::cos(roll) + sin(yaw)*sin(roll);
    rotaiton_matrix (1,2) = sin(yaw)*sin(pitch)*std::cos(roll) - std::cos(yaw)*sin(roll);
    rotaiton_matrix (2,2) = std::cos(pitch)*std::cos(roll);

    return rotaiton_matrix;

}

std::vector<std::string> get_pose(std::string path, std::string frame_number){
    
    std::ifstream pose_file(path);
    std::string line;

    while(getline (pose_file, line)) 
    {   
        std::vector<std::string> split_line = splitString(line, ',');
        if (frame_number == split_line[0]) 
            return split_line;
    }

}
