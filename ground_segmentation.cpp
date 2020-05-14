//
// Created by chli on 11.05.20.
//
#include "ground_segmentation.h"

void ground_segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
{

  pcl::PointIndicesPtr ground (new pcl::PointIndices);


  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *cloud << std::endl;

  // Create the filtering object
  pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
  pmf.setInputCloud (cloud);
  std::cout << "1!!!!!!!!!!"<<std::endl;
  pmf.setMaxWindowSize (20);
  std::cout << "2!!!!!!!!!!"<<std::endl;
  pmf.setSlope (1.0f);
  std::cout << "3!!!!!!!!!!"<<std::endl;
  pmf.setInitialDistance (0.5f);
  std::cout << "4!!!!!!!!!!"<<std::endl;
  pmf.setMaxDistance (3.0f);
  std::cout << "5!!!!!!!!!!"<<std::endl;
  pmf.extract (ground->indices);
  std::cout << "!!!!!!!!!!"<<std::endl;

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  std::cout << "6!!!!!!!!!!"<<std::endl;
  extract.setInputCloud (cloud);
  std::cout << "7!!!!!!!!!!"<<std::endl;
  extract.setIndices (ground);
  std::cout << "8!!!!!!!!!!"<<std::endl;
  extract.filter (*cloud_filtered);
  std::cout << "9!!!!!!!!!!"<<std::endl;

  std::cerr << "Ground cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;

}
