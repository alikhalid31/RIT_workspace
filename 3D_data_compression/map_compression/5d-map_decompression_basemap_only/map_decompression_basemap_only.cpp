#include <iostream>
#include <fstream>
#include <filesystem>
#include <vector>  
#include <string.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/octree/octree_search.h>
#include <pcl/filters/extract_indices.h>
#include <ctime>


// Usage 
// ./map_compression [datset folder] [basemap_path]  [day2 (day to be compressed)] [resolution of octree] [input folder] [output folder]
 
int recover (std::string, std::string, std::vector<std::string>, pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr, float, pcl::ExtractIndices<pcl::PointXYZ>&, 
std::vector<float>&, std::vector<float>&, std::vector<float>& );
std::vector<std::string> splitString(std::string , char );
int decompress_frames (std::string, std::string,  float, std::string, std::string);
Eigen::Matrix4f convert_YPR_to_rotation_matrix(float, float, float);
Eigen::Matrix4f get_transformation_matrix(std::vector<std::string> );
std::vector<std::string> get_pose(std::string , std::string );



int main (int argc, char** argv)

{
    std::string dataset_folder = std::string(argv[1]);
    std::string basemap_path = std::string(argv[2]);
    std::string compress_day_path = dataset_folder + "/" + std::string(argv[3]);
    float resolution = atof(argv[4]);
    std::string input_folder = std::string(argv[5]);
    std::string output_folder = std::string(argv[6]);



    decompress_frames(compress_day_path, basemap_path, resolution, input_folder, output_folder);

    return (0);
}


int decompress_frames (std::string compress_day_path, std::string basemap_path,  float resolution, std::string input_folder, std::string output_folder)
{
 
    std::vector<float> run_time_1, run_time_2, run_time_3, run_time_4;
    std::vector<float> run_time_total_wload, run_time_total_woload;
    float avg_wload, avg_woload;

    pcl::PointCloud<pcl::PointXYZ>::Ptr recovered (new pcl::PointCloud<pcl::PointXYZ> );


    // loding base map point cloud and downsampling it 
    pcl::PointCloud<pcl::PointXYZ>::Ptr base_cloud (new pcl::PointCloud<pcl::PointXYZ> );
    pcl::io::loadPCDFile<pcl::PointXYZ>(basemap_path+"/complete_pcl.pcd", *base_cloud);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (base_cloud);
    extract.setNegative (false);


    std::string line;
    std::ifstream association_file(compress_day_path+"/pose.txt");

    while (getline (association_file, line )) 
    {
        std::vector<std::string> split_line = splitString(line, ',');
        std::vector<std::string> compress_frame_pose = split_line;
        std::string compress_data_path = compress_day_path + "/" +input_folder ;
        std::string compress_frame = split_line[0];
        //std::cout << split_line[0] <<std::endl;

        if (recover (compress_data_path, compress_frame, compress_frame_pose,  base_cloud, recovered, resolution, extract, run_time_1, run_time_2, run_time_3 ) == 0 )
        {

            std::string recover_result_folder = compress_day_path + "/" + output_folder  ;

            if (!std::filesystem::is_directory(recover_result_folder) || !std::filesystem::exists(recover_result_folder)) 
            { 
                std::filesystem::create_directories(recover_result_folder);
            }
            pcl::io::savePCDFileASCII (recover_result_folder+"/"+ split_line[0] +".pcd", *recovered);
   
            run_time_4.push_back(clock());

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
    return (0);
}

int recover (std::string compress_day_path, std::string compress_frame, std::vector<std::string> compress_frame_pose, pcl::PointCloud<pcl::PointXYZ>::Ptr base_cloud, 
pcl::PointCloud<pcl::PointXYZ>::Ptr recovered, float resolution, pcl::ExtractIndices<pcl::PointXYZ>& extract,
std::vector<float>& run_time_1, std::vector<float>& run_time_2, std::vector<float>& run_time_3)
{
    run_time_1.push_back(clock());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_add (new pcl::PointCloud<pcl::PointXYZ>);

    std::vector<int> static_points;
    std::ifstream fin(compress_day_path + "/remove_indices/" + compress_frame + ".dat", std::fstream::binary | std::fstream::out);
    int num;
    while (fin >> num)
        static_points.push_back(num);


    Eigen::Matrix4f transformation_matrix = get_transformation_matrix(compress_frame_pose);




    

    
    run_time_2.push_back(clock());


    inliers->indices = static_points;
    extract.setIndices (inliers);
    extract.filter (*recovered);


    pcl::transformPointCloud (*recovered, *recovered, transformation_matrix.inverse());

    if (pcl::io::loadPCDFile<pcl::PointXYZ> (compress_day_path+"/add/"+compress_frame+".pcd", *cloud_add) != -1) //* load the file
    {
        //PCL_ERROR ("Couldn't read file day 1  \n");
        *recovered += *cloud_add;
        //return (-1);
    }

    //*recovered += *cloud_add;

    run_time_3.push_back(clock());


    return(0);
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

