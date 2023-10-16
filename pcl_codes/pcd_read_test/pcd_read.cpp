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
//Commnets : Hunjung, Lim (hunjung.lim@hotmail.com)
// ./pcd_read /dataset/input/w_noise/ base/20s/1/ base/base_w_noise_objects car1/20s/1/ 1 1  20 0.1 0.2 100 1 1


int main (int argc, char** argv)
{ 
  clock_t  clock1,clock2,clock3,clock4,clock5;

  pcl::PointCloud<pcl::PointXYZ>::Ptr base_cloud (new pcl::PointCloud<pcl::PointXYZ> );
  pcl::PointCloud<pcl::PointXYZ>::Ptr base_cloud_objects (new pcl::PointCloud<pcl::PointXYZ> );
  pcl::PointCloud<pcl::PointXYZ>::Ptr ego_cloud (new pcl::PointCloud<pcl::PointXYZ> );
  pcl::PointCloud<pcl::PointXYZ>::Ptr base_filterted (new pcl::PointCloud<pcl::PointXYZ> );
  pcl::PointCloud<pcl::PointXYZ>::Ptr change_A (new pcl::PointCloud<pcl::PointXYZ> );
  pcl::PointCloud<pcl::PointXYZ>::Ptr change_B (new pcl::PointCloud<pcl::PointXYZ> );
  pcl::PointCloud<pcl::PointXYZ>::Ptr total_change (new pcl::PointCloud<pcl::PointXYZ> );
  pcl::PointCloud<pcl::PointXYZ>::Ptr total_change_A (new pcl::PointCloud<pcl::PointXYZ> );
  pcl::PointCloud<pcl::PointXYZ>::Ptr total_change_B (new pcl::PointCloud<pcl::PointXYZ> );
  
  std::vector<float> run_time;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  pcl::ExtractIndices<pcl::PointXYZ> extract;


  // reading command line aarguments
  std::string input_dir = std::string(argv[1]);
  std::string base = std::string(argv[2]);
  std::string base_objects = std::string(argv[3]);

  std::string vehicle = std::string(argv[4]);
  int counter = 1;
  int start_id = atoi(argv[5]); //side length of octree voxels
  int end_id = atoi(argv[6]);
  float radius = atof(argv[7]);
  float resolution_base = atof(argv[8]);
  float resolution_voxel = atof(argv[9]);
  int outlier_K = atoi(argv[10]);
  float outlier_std = atof(argv[11]);
  int interval = atoi(argv[12]);
  float sum_of_elems = 0.0;
  

  // loding base map point cloud and downsampling it 
  pcl::io::loadPCDFile<pcl::PointXYZ>(input_dir+base+".pcd", *base_cloud);
  pcl::VoxelGrid<pcl::PointXYZ> ds;
  ds.setInputCloud (base_cloud);
  ds.setLeafSize (resolution_base, resolution_base, resolution_base);
  ds.filter(*base_cloud);
  // pcl::io::savePCDFileASCII ("/dataset/change_detection/output/base_downsampled.pcd", *base_cloud);

  // loding base map objects point cloud and downsampling it 
  // pcl::io::loadPCDFile<pcl::PointXYZ>(input_dir+base_objects+".pcd", *base_cloud_objects);
  // ds.setInputCloud (base_cloud_objects);
  // ds.setLeafSize (resolution_base, resolution_base, resolution_base);
  // ds.filter(*base_cloud_objects);

  // pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree_base_objects (resolution_voxel);
  // octree_base_objects.setInputCloud (base_cloud);
  // octree_base_objects.addPointsFromInputCloud ();
  // octree_base_objects.switchBuffers ();
  // octree_base_objects.setInputCloud (base_cloud_objects);
  // octree_base_objects.addPointsFromInputCloud ();

  // std::vector<int> newPointIdxVector;
  // octree_base_objects.getPointIndicesFromNewVoxels (newPointIdxVector);
  // inliers->indices = newPointIdxVector;
  // extract.setInputCloud (base_cloud_objects);
  // extract.setIndices (inliers);
  // extract.setNegative (false);
  // extract.filter (*base_cloud_objects);
  //pcl::io::savePCDFileASCII ("/dataset/output/change_detection/frameVSmap/base_objects.pcd", *base_cloud_objects);


  // create octree of a base cloud for filtering points within radius 
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree_filter (resolution_voxel);
  octree_filter.setInputCloud (base_cloud);
  octree_filter.addPointsFromInputCloud ();


  pcl::PointCloud<pcl::PointXYZ>::Ptr ego_cloud_acc (new pcl::PointCloud<pcl::PointXYZ> );

  for (int i = start_id; i <= end_id ; i++) {

    std::cout << "working on " << i << ".pcd" << std::endl;



    pcl::io::loadPCDFile<pcl::PointXYZ>(input_dir+vehicle+std::to_string(i)+".pcd", *ego_cloud);
    

    if (counter%interval == 0){

      *ego_cloud_acc += *ego_cloud;

      // compute centeroid of the accumulatie ego cloud 

      pcl::PointXYZ centroid_ego; 
      std::vector<int> pointIdxRadiusSearch, pointIdxRadiusSearch_A;
      std::vector<float> pointRadiusSquaredDistance, pointRadiusSquaredDistance_A;
      pcl::computeCentroid (*ego_cloud_acc, centroid_ego);
      std::cout << centroid_ego <<std::endl;

      // filter the point cloud to keep points within radius
      octree_filter.radiusSearch (centroid_ego, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);

      pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree_filter_ego (resolution_voxel);
      octree_filter_ego.setInputCloud (ego_cloud_acc);
      octree_filter_ego.addPointsFromInputCloud ();
      octree_filter_ego.radiusSearch (centroid_ego, radius, pointIdxRadiusSearch_A, pointRadiusSquaredDistance_A);

      
      inliers->indices = pointIdxRadiusSearch;
      extract.setInputCloud (base_cloud);
      extract.setIndices (inliers);
      extract.setNegative (false);
      extract.filter (*base_filterted);
      pcl::io::savePCDFileASCII ("/dataset/output/change_detection/frameVSmap/"+std::to_string(i)+"_base_R.pcd", *base_filterted);

      inliers->indices = pointIdxRadiusSearch_A;
      extract.setInputCloud (ego_cloud_acc);
      extract.setIndices (inliers);
      extract.setNegative (false);
      extract.filter (*ego_cloud_acc);
      pcl::io::savePCDFileASCII ("/dataset/output/change_detection/frameVSmap/"+std::to_string(i)+"_ego_R.pcd", *ego_cloud_acc);
      


      // find change A
      pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree_change_A (resolution_voxel);
      octree_change_A.setInputCloud (base_filterted);
      octree_change_A.addPointsFromInputCloud ();
      octree_change_A.switchBuffers ();
      octree_change_A.setInputCloud (ego_cloud_acc);
      octree_change_A.addPointsFromInputCloud ();

      std::vector<int> newPointIdxVector_A;
      octree_change_A.getPointIndicesFromNewVoxels (newPointIdxVector_A);
      inliers->indices = newPointIdxVector_A;
      extract.setInputCloud (ego_cloud_acc);
      extract.setIndices (inliers);
      extract.setNegative (false);
      extract.filter (*change_A);

      pcl::io::savePCDFileASCII ("/dataset/output/change_detection/frameVSmap/"+std::to_string(i)+"_changeA.pcd", *change_A);

      
      pcl::ModelCoefficients::Ptr coefficients1 (new pcl::ModelCoefficients);
      pcl::SACSegmentation<pcl::PointXYZ> seg1;
      // Optional
      seg1.setOptimizeCoefficients (true);
      // Mandatory
      seg1.setModelType (pcl::SACMODEL_PLANE);
      seg1.setMethodType (pcl::SAC_RANSAC);
      seg1.setDistanceThreshold (0.1);
      seg1.setInputCloud (change_A);
      seg1.segment (*inliers, *coefficients1);

      extract.setInputCloud (change_A);
      extract.setIndices (inliers);
      extract.setNegative (true);
      extract.filter (*change_A);
      pcl::io::savePCDFileASCII ("/dataset/output/change_detection/frameVSmap/"+std::to_string(i)+"_plane_changeA.pcd", *change_A);


      *total_change_A += *change_A ;



      pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor1;
      sor1.setInputCloud (change_A);
      sor1.setMeanK (outlier_K);
      sor1.setStddevMulThresh (outlier_std);
      sor1.filter (*change_A);

      pcl::io::savePCDFileASCII ("/dataset/output/change_detection/frameVSmap/"+std::to_string(i)+"_stat_changeA.pcd", *change_A);





      // find change B
      pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree_change_B (resolution_voxel);
      octree_change_B.setInputCloud (ego_cloud_acc);
      octree_change_B.addPointsFromInputCloud ();
      octree_change_B.switchBuffers ();
      octree_change_B.setInputCloud (base_filterted);
      octree_change_B.addPointsFromInputCloud ();

      std::vector<int> newPointIdxVector_B;
      octree_change_B.getPointIndicesFromNewVoxels (newPointIdxVector_B);
      inliers->indices = newPointIdxVector_B;
      extract.setInputCloud (base_filterted);
      extract.setIndices (inliers);
      extract.setNegative (false);
      extract.filter (*change_B);

      pcl::io::savePCDFileASCII ("/dataset/output/change_detection/frameVSmap/"+std::to_string(i)+"_changeB.pcd", *change_B);


      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      pcl::SACSegmentation<pcl::PointXYZ> seg;
      // Optional
      seg.setOptimizeCoefficients (true);
      // Mandatory
      seg.setModelType (pcl::SACMODEL_PLANE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setDistanceThreshold (0.1);
      seg.setInputCloud (change_B);
      seg.segment (*inliers, *coefficients);

      extract.setInputCloud (change_B);
      extract.setIndices (inliers);
      extract.setNegative (true);
      extract.filter (*change_B);

      pcl::io::savePCDFileASCII ("/dataset/output/change_detection/frameVSmap/"+std::to_string(i)+"_plane_changeB.pcd", *change_B);


      pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
      sor.setInputCloud (change_B);
      sor.setMeanK (outlier_K);
      sor.setStddevMulThresh (outlier_std);
      sor.filter (*change_B);

      pcl::io::savePCDFileASCII ("/dataset/output/change_detection/frameVSmap/"+std::to_string(i)+"_stat_changeB.pcd", *change_B);


      *total_change_B += *change_B ;

    }
    else if (counter%interval == 1){


      *ego_cloud_acc = *ego_cloud;

    }
    else  {
      *ego_cloud_acc += *ego_cloud;

    }
    counter+=1;
    

  }
 
      
  // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  // pcl::SACSegmentation<pcl::PointXYZ> seg;
  // // Optional
  // seg.setOptimizeCoefficients (true);
  // // Mandatory
  // seg.setModelType (pcl::SACMODEL_PLANE);
  // seg.setMethodType (pcl::SAC_RANSAC);
  // seg.setDistanceThreshold (0.1);
  // seg.setInputCloud (total_change_A);
  // seg.segment (*inliers, *coefficients);

  // extract.setInputCloud (total_change_A);
  // extract.setIndices (inliers);
  // extract.setNegative (true);
  // extract.filter (*total_change_A);

  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (total_change_A);
  sor.setMeanK (outlier_K);
  sor.setStddevMulThresh (outlier_std);
  sor.filter (*total_change_A);


  // std::cout << "average time: " << (float)sum_of_elems/run_time.size()<< std::endl;
  // pcl::io::savePCDFileASCII ("/dataset/output/change_detection/frameVSmap/totalChange.pcd", *total_change);
  // pcl::io::savePCDFileASCII ("/dataset/output/change_detection/frameVSmap/changeA.pcd", *total_change_A);
  ds.setInputCloud (total_change_B);
  ds.setLeafSize (resolution_base, resolution_base, resolution_base);
  ds.filter(*total_change_B);
  //pcl::io::savePCDFileASCII ("/dataset/output/change_detection/frameVSmap/changeB.pcd", *total_change_B);
  //pcl::io::savePCDFileASCII ("/dataset/output/change_detection/frameVSmap/changeA.pcd", *total_change_A);


 
  




  // // concatanating point clouds and saving them
  // *change_A += *change_B;
  // clock5 = clock();
  // std::cout<< "Execution Time ego-base: "<< (float)(clock3 - clock2)/ CLOCKS_PER_SEC << " "<<std::endl;
  // std::cout<< "Execution Time base-ego: "<< (float)(clock4 - clock3)/ CLOCKS_PER_SEC << " "<<std::endl;
  // std::cout<< "Total Execution Time: "<< (float)(clock5 - clock1)/ CLOCKS_PER_SEC << " "<<std::endl;
  

}

/////////////////////// Conversion Code /////////////////////////
  // pcl::toPCLPointCloud2(*base, *base2);
  // pcl::fromPCLPointCloud2(*base2, *base); 


////////////////////// Printing Result /////////////////////////
 // for (std::size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
    // std::cout << "    "  <<   (*base)[ pointIdxRadiusSearch[i] ].x << " " << (*base)[ pointIdxRadiusSearch[i] ].y << " " << (*base)[ pointIdxRadiusSearch[i] ].z << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
