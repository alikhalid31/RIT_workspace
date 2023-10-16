#include <pcl/io/pcd_io.h> 
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/octree/octree_search.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <iostream>
#include <vector>
#include <ctime>
#include <pcl/conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>



//Spatial change detection on unorganized point cloud data
//http://pointclouds.org/documentation/tutorials/octree_change.php#octree-change-detection 
//Commnets : ali_khalid
// ./raw_framevsframe_change /dataset/input/wo_noise/ base/20s/1/ car1/20s/3/ 1 199 0 0.1


int main (int argc, char** argv)
{ 
  clock_t  clock1,clock2,clock3,clock4,clock5;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1 (new pcl::PointCloud<pcl::PointXYZ> );
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2 (new pcl::PointCloud<pcl::PointXYZ> );
  pcl::PointCloud<pcl::PointXYZ>::Ptr change_addition (new pcl::PointCloud<pcl::PointXYZ> );
  pcl::PointCloud<pcl::PointXYZ>::Ptr change_removal (new pcl::PointCloud<pcl::PointXYZ> );
  pcl::PointCloud<pcl::PointXYZ>::Ptr total_change_addition (new pcl::PointCloud<pcl::PointXYZ> );
  pcl::PointCloud<pcl::PointXYZ>::Ptr total_change_removal (new pcl::PointCloud<pcl::PointXYZ> );
  

  // reading command line aarguments
  std::string input_dir = std::string(argv[1]);
  std::string cloud1_dir = std::string(argv[2]);
  std::string cloud2_dir = std::string(argv[3]); 

  int start_id = atoi(argv[4]); //side length of octree voxels
  int end_id = atoi(argv[5]);
  float radius = atof(argv[6]);
  float resolution = atof(argv[7]);
  float sum_of_elems = 0.0;



  std::vector<float> run_time;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  for (int i = start_id; i <= end_id ; i++) {
    std::cout << "working on " << i << ".pcd"; 
    pcl::io::loadPCDFile<pcl::PointXYZ>(input_dir+cloud1_dir+std::to_string(i)+".pcd", *cloud_1);
    pcl::io::loadPCDFile<pcl::PointXYZ>(input_dir+cloud2_dir+std::to_string(i)+".pcd", *cloud_2);

    if (radius != 0){
      pcl::PointXYZ centroid_1, centroid_2; 
      pcl::computeCentroid (*cloud_1, centroid_1);
      pcl::computeCentroid (*cloud_2, centroid_2);

      std::vector<int> pointIdxRadiusSearch_1, pointIdxRadiusSearch_2;
      std::vector<float> pointRadiusSquaredDistance_1, pointRadiusSquaredDistance_2;


      pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree_filter_1 (resolution);
      octree_filter_1.setInputCloud (cloud_1);
      octree_filter_1.addPointsFromInputCloud ();
      octree_filter_1.radiusSearch (centroid_1, radius, pointIdxRadiusSearch_1, pointRadiusSquaredDistance_1);
      inliers->indices = pointIdxRadiusSearch_1;
      extract.setInputCloud (cloud_1);
      extract.setIndices (inliers);
      extract.setNegative (false);
      extract.filter (*cloud_1);

      pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree_filter_2 (resolution);
      octree_filter_2.setInputCloud (cloud_2);
      octree_filter_2.addPointsFromInputCloud ();
      octree_filter_2.radiusSearch (centroid_2, radius, pointIdxRadiusSearch_2, pointRadiusSquaredDistance_2);
      inliers->indices = pointIdxRadiusSearch_2;
      extract.setInputCloud (cloud_2);
      extract.setIndices (inliers);
      extract.setNegative (false);
      extract.filter (*cloud_2);
      
    }

    clock1 = clock();
    // ADDITION OF POINTS
    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree_change_addition(resolution);
    octree_change_addition.setInputCloud (cloud_1);
    octree_change_addition.addPointsFromInputCloud ();
    octree_change_addition.switchBuffers ();
    octree_change_addition.setInputCloud (cloud_2);
    octree_change_addition.addPointsFromInputCloud ();

    std::vector<int> newPointIdxVector_addition;
    octree_change_addition.getPointIndicesFromNewVoxels (newPointIdxVector_addition);
    inliers->indices = newPointIdxVector_addition;
    extract.setInputCloud (cloud_2);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*change_addition);

    // REMOVAL OF POINTS
    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree_change_removal (resolution);
    octree_change_removal.setInputCloud (cloud_2);
    octree_change_removal.addPointsFromInputCloud ();
    octree_change_removal.switchBuffers ();
    octree_change_removal.setInputCloud (cloud_1);
    octree_change_removal.addPointsFromInputCloud ();

    std::vector<int> newPointIdxVector_removal;
    octree_change_removal.getPointIndicesFromNewVoxels (newPointIdxVector_removal);
    inliers->indices = newPointIdxVector_removal;
    extract.setInputCloud (cloud_1);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*change_removal);
    clock2 = clock();


    std::cout<< ", time taken: " << (float)(clock2-clock1)/CLOCKS_PER_SEC <<std::endl;
    run_time.push_back((float)(clock2-clock1)/CLOCKS_PER_SEC);

    *total_change_addition += *change_addition;
    *total_change_removal += *change_removal;


  }

  for(std::vector<float>::iterator it = run_time.begin(); it != run_time.end(); ++it)
    sum_of_elems += *it;
  
  std::cout << "average time: " << (float)sum_of_elems/run_time.size()<< std::endl;
  pcl::io::savePCDFileASCII ("/dataset/output/change_detection/frameVSframe/raw/change_addition.pcd", *total_change_addition);
  pcl::io::savePCDFileASCII ("/dataset/output/change_detection/frameVSframe/raw/change_removal.pcd", *total_change_removal);

}

/////////////////////// Conversion Code /////////////////////////
  // pcl::toPCLPointCloud2(*base, *base2);
  // pcl::fromPCLPointCloud2(*base2, *base); 


////////////////////// Printing Result /////////////////////////
 // for (std::size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
    // std::cout << "    "  <<   (*base)[ pointIdxRadiusSearch[i] ].x << " " << (*base)[ pointIdxRadiusSearch[i] ].y << " " << (*base)[ pointIdxRadiusSearch[i] ].z << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
