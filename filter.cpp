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
#include <pcl/surface/mls.h>

#include "filter.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

void radius_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
{

    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    // build the filter
    outrem.setInputCloud(cloud);
    outrem.setRadiusSearch(0.5);
    outrem.setMinNeighborsInRadius (5);
    // apply filter
    outrem.filter (*cloud_filtered);

}

void voxel_grid (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
{

    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
              << " data points (" << pcl::getFieldsList (*cloud) << ").";

    // Create the filtering object
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (0.1f, 0.1f, 0.1f);
    sor.filter (*cloud_filtered);

    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
              << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")."<< std::endl;

    //pcl::PCDWriter writer;
    //writer.write ("table_scene_lms400_downsampled.pcd", *cloud_filtered,
    //              Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

}

void resampling(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        pcl::PointCloud<pcl::PointNormal>::Ptr mls_points)
{
    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
              << " data points (" << pcl::getFieldsList (*cloud) << ").";
    // Create a KD-Tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
    mls.setComputeNormals (true);

    // Set parameters
    mls.setInputCloud (cloud);
    mls.setPolynomialOrder (2);
    mls.setSearchMethod (tree);
    mls.setSearchRadius (0.3);

    // Reconstruct
    mls.process (*mls_points);
    std::cerr << "PointCloud after filtering: " << mls_points->width * mls_points->height
              << " data points (" << pcl::getFieldsList (*mls_points) << ")."<<std::endl;
}


void normal_estimation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered,
                       pcl::PointCloud<pcl::Normal>::Ptr normals)
{
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud_filtered);
    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    //tree->setInputCloud (cloud_filtered);
    ne.setSearchMethod (tree);
    ne.setRadiusSearch (0.1);
    ne.setKSearch (20);
    ne.compute (*normals);


}

void normal_estimation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered,
                       pcl::PointCloud<pcl::Normal>::Ptr normals,
                       pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals)
{
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud_filtered);
    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    //tree->setInputCloud (cloud_filtered);
    ne.setSearchMethod (tree);
    //ne.setRadiusSearch (0.1);
    ne.setKSearch (20);
    ne.compute (*normals);
    // Concatenate the XYZ and normal fields*

    pcl::concatenateFields (*cloud_filtered, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals

}



void normal_estimation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered,
                       pcl::PointCloud<pcl::Normal>::Ptr normals_small_scale,
                       pcl::PointCloud<pcl::Normal>::Ptr normals_large_scale)
{
    // normal estimation
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud_filtered);
    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);
    ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());
    ne.setRadiusSearch (0.1);
    ne.compute (*normals_small_scale);

    // Output datasets
    ne.setRadiusSearch (0.3);
    ne.compute (*normals_large_scale);
}





/*
// Create output cloud for DoN results
pcl::PointCloud<pcl::Normal>::Ptr doncloud (new pcl::PointCloud<pcl::Normal>);
copyPointCloud (*cloud, *doncloud);


//生成DoN分割器
pcl::DifferenceOfNormalsEstimation<pcl::PointXYZ, pcl::Normal, pcl::Normal> don;
don.setInputCloud (cloud);
don.setNormalScaleLarge (normals_large_scale);
don.setNormalScaleSmall (normals_small_scale);

//计算法线差
don.computeFeature (*doncloud);

// Build the condition for filtering
pcl::ConditionOr<pcl::Normal>::Ptr range_cond (
        new pcl::ConditionOr<pcl::Normal> ()
);
range_cond->addComparison (pcl::FieldComparison<pcl::Normal>::ConstPtr (
        new pcl::FieldComparison<pcl::Normal> ("curvature", pcl::ComparisonOps::GT, 5))

// Build the filter
pcl::ConditionalRemoval<pcl::Normal> condrem;
condrem.setCondition (range_cond);
condrem.setInputCloud (doncloud);

pcl::PointCloud<pcl::Normal>::Ptr doncloud_filtered (new pcl::PointCloud<pcl::Normal>);

// Apply filter
condrem.filter (*doncloud_filtered);

doncloud = doncloud_filtered;

// Save filtered output
std::cout << "Filtered Pointcloud: " << doncloud->points.size () << " data points." << std::endl;

writer.write<pcl::PointNormal> ("don_filtered.pcd", *doncloud, false);
 */