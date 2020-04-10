#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "filter.h"
#include "surface.h"
#include "visualization.h"
#include <pcl/visualization/cloud_viewer.h>
#include "range_image.h"
#include <pcl/io/vtk_io.h>
#include "preprocessing.h"

int main ()
{
    //preprocessing();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZ>);
    // load the cloud data
    pcl::io::loadPCDFile ("./data/0356_3_road_new.pcd", *cloud);
    radius_filter(cloud, cloud_filtered);
    pcl::PCDWriter writer;
    //writer.write<pcl::PointXYZ>("./data/0356_3_road_filter_new.pcd", *cloud_filtered, false);

    /* //usual normal estimation
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    normal_estimation(cloud_filtered, normals, cloud_with_normals );
    */
    voxel_grid(cloud_filtered,cloud_filtered2);
    visualization(cloud_filtered2);

    // resamling filter
    pcl::PointCloud<pcl::PointNormal>::Ptr mls_points (new pcl::PointCloud<pcl::PointNormal>);
    resampling(cloud_filtered2, mls_points);
    //pcl::io::savePCDFile ("bun0-mls.pcd", mls_points);


    // greedy triangularation
    pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh);
    greedy_triangulation(mls_points,triangles);
    pcl::io::saveVTKFile ("./data/0356_3_road.vtk", *triangles);
    //range_image2(cloud_filtered);



    //marchingcubes reconstruction
    //marchingcubes (cloud_with_normals);

    //visualization(cloud_filtered);
    //normalsVis(cloud_filtered, normals);

    //range_image(cloud_filtered);

    return (0);
}


