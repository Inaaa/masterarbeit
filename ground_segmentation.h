
//
// Created by chli on 11.05.20.
//
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>


#ifndef MASTERARBEIT_GROUND_SEGMENTATION_H
#define MASTERARBEIT_GROUND_SEGMENTATION_H

void ground_segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered);

#endif //MASTERARBEIT_GROUND_SEGMENTATION_H
