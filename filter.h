//
// Created by chli on 09.04.20.
//
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
//#include <pcl/features/normal_3d.h>
#include <pcl/features/don.h>
#include <pcl/features/normal_3d_omp.h>


#ifndef MASTERARBEIT_FILTER_H
#define MASTERARBEIT_FILTER_H

void radius_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered);

void resampling(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                pcl::PointCloud<pcl::PointNormal> mls_points);

void normal_estimation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered,
                       pcl::PointCloud<pcl::Normal>::Ptr normals);


void normal_estimation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered,
                       pcl::PointCloud<pcl::Normal>::Ptr normals,
                       pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals);


void normal_estimation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered,
                       pcl::PointCloud<pcl::Normal>::Ptr normals_small_scale,
                       pcl::PointCloud<pcl::Normal>::Ptr normals_large_scale);

void range_image();


#endif //MASTERARBEIT_FILTER_H
