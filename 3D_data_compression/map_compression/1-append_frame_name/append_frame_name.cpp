#include <iostream>
#include <fstream>
#include <filesystem>
#include <vector>  
#include <string.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


//usasge 
// ./append_frame_name [Path to dataset folder]
// the dataset folder should contain subfolders for each day as shown bellow  
// - dataset
//      - day1
//          - pcds 
//              - 1.pcd
//              - 2.pcd
//          - pose.txt
//      - day2
//          - pcds 
//              - 1.pcd
//              - 2.pcd
//          - pose.txt


int append_attribute_per_pcd(std::string);
int append_attribute_per_folder(std::string);
std::vector<std::string> splitString(std::string , char );
std::vector<std::string> get_directories(std::string&);



int main (int argc, char** argv)
{
    std::string dataset_folder = std::string(argv[1]);

    std::vector<std::string> dirs = get_directories(dataset_folder);
    for (auto i: dirs)
    {
        std::cout << i << std::endl;
        append_attribute_per_folder(i);
    }

    return (0);
}

int append_attribute_per_folder(std::string foldername)
{

    for (const auto & entry : std::filesystem::directory_iterator(foldername+"/pcds"))
    {
        std::cout << entry.path() << std::endl;
        append_attribute_per_pcd(entry.path());
    }


    return(0);
}

int append_attribute_per_pcd(std::string frame_path)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    std::vector<std::string> split_1 = splitString(frame_path, '/');
    //std::vector<std::string> frame_name = splitString(split_1.back(), '.');
    std::string frame_name = split_1.back();//.substr(0,split_1.size());
    //std::cout << frame_name << std::endl;

    if (pcl::io::loadPCDFile<pcl::PointXYZI> (frame_path, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    //std::cout << "Loaded " << cloud->width * cloud->height << " data points from test_pcd.pcd with the following fields: " << std::endl;

    for (auto& point: *cloud)
        point.intensity = std::stof(frame_name);

    pcl::io::savePCDFileASCII (frame_path, *cloud);

    return(0);

}

std::vector<std::string> get_directories(std::string& s)
{
    std::vector<std::string> r;
    for(auto& p : std::filesystem::directory_iterator(s))
        if (p.is_directory())
            r.push_back(p.path().string());
    return r;
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
