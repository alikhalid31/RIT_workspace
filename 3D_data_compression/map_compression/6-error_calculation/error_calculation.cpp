#include <iostream>
#include <vector> 
#include <pcl/io/pcd_io.h>
#include<pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <filesystem>

// Usage 
// .error_calculation [dataset path ] [day for which error is to be calculated ] [reconstructed pcds folder] [K] [ mode( O (for ply frame by frame) or 1 (for pcd frame by frame))]

float compute (pcl::PointCloud<pcl::PointXYZ>::Ptr , pcl::PointCloud<pcl::PointXYZ>::Ptr , int );
std::vector<std::string> get_directories(std::string&);

pcl::PointCloud<pcl::PointXYZ>::Ptr infra_static (new pcl::PointCloud<pcl::PointXYZ>);


int main (int argc, char** argv)
{
  std::string data_path = argv[1];
  std::string day = argv[2];
  std::string reconstructed_pcds_folder = argv[3];
  int K = atoi(argv[4]);
  int mode = atoi(argv[5]);


  if (pcl::io::loadPCDFile<pcl::PointXYZ> (data_path+"/infra_static.pcd", *infra_static) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read infra-ststic \n");
      return (-1);
    }

  if (mode == 0)
  {
    std::string source_pcd_folder = data_path + '/' + day + '/'+ "plys";
    std::string target_pcd_folder = data_path + '/' + day + '/'+ reconstructed_pcds_folder;

    float avg_rmse = 0.0f;
    int count = 0;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target (new pcl::PointCloud<pcl::PointXYZ>);


    for (const auto & entry : std::filesystem::directory_iterator(source_pcd_folder))
    {
      //std::cout << entry.path() << std::endl;
      std::string  path = entry.path();
      pcl::PLYReader Reader;

      if (Reader.read(entry.path(), *cloud_source) == -1) //* load the file
      {
        PCL_ERROR ("Couldn't read file day 1  \n");
        return (-1);
      }
      if (Reader.read(target_pcd_folder  + '/' + path.substr(path.find_last_of("/\\") + 1) , *cloud_target) == -1) //* load the file
      {
        PCL_ERROR ("Couldn't read file day 1  \n");
        return (-1);
      }
      avg_rmse += compute (cloud_source, cloud_target, K);
      count += 1;
    }

    avg_rmse = avg_rmse/ (float) count;
    std::cout << reconstructed_pcds_folder + ": " ;
    std::cout << avg_rmse << std::endl;

  }

  else 
  {
    std::string source_pcd_folder = data_path + '/' + day; // + '/'+ "pcds";
    std::string target_pcd_folder = data_path + '/' + reconstructed_pcds_folder;

    float avg_rmse = 0.0f;
    int count = 0;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target (new pcl::PointCloud<pcl::PointXYZ>);


    for (const auto & entry : std::filesystem::directory_iterator(source_pcd_folder))
    {
      std::cout << entry.path() << std::endl;
      std::string  path = entry.path();


      if (pcl::io::loadPCDFile<pcl::PointXYZ> (entry.path(), *cloud_source) == -1) //* load the file
      {
        PCL_ERROR ("Couldn't read file day 1  \n");
        return (-1);
      }
      if (pcl::io::loadPCDFile<pcl::PointXYZ> (target_pcd_folder  + '/' + path.substr(path.find_last_of("/\\") + 1) , *cloud_target) == -1) //* load the file
      {
        PCL_ERROR ("Couldn't read file day 1  \n");
        return (-1);
      }
      *cloud_target = *cloud_target + *infra_static;
      avg_rmse += compute (cloud_source, cloud_target, K);
      count += 1;
    }

    avg_rmse = avg_rmse/ (float) count;
    std::cout << reconstructed_pcds_folder + ": " ;
    std::cout << avg_rmse << std::endl;

  }
  return 0;
}

float compute (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target, int K)
{

  float rmse = 0.0f;
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud (cloud_target);

  for (std::size_t point_i = 0; point_i < cloud_source->size(); ++ point_i)
  {
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    if ( tree.nearestKSearch ((*cloud_source)[point_i], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
    {
      for (std::size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
      {
        rmse += pointNKNSquaredDistance[i];
      }
      
    }
  }

  rmse = std::sqrt (rmse / (float)cloud_source->size());
  //std::cout << rmse << std::endl;

  return rmse;

}

std::vector<std::string> get_directories(std::string& s)
{
    std::vector<std::string> r;
    for(auto& p : std::filesystem::directory_iterator(s))
        if (p.is_directory())
            r.push_back(p.path().string());
    return r;
}


