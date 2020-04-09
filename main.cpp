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
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include "filter.h"
#include "surface.h"
#include "visualization.h"
#include <pcl/visualization/cloud_viewer.h>
#include "range_image.h"


int main ()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    // load the cloud data
    pcl::io::loadPCDFile ("/mrtstorage/users/students/chli/real_data/test_data2/road_new.pcd", *cloud);
    radius_filter(cloud, cloud_filtered);
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ>("/mrtstorage/users/chli/real_data/test_data2/road_filter_new.pcd", *cloud_filtered, false);

    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    normal_estimation(cloud_filtered, normals, cloud_with_normals );

    // greedy triangularation
    //pcl::PolygonMesh triangles = greedy_triangulation(cloud_with_normals);
    //pcl::io::saveVTKFile ("/mrtstorage/users/chli/real_data/test_data2/road.vtk", triangles);
    range_image2(cloud_filtered);
    pcl::PointCloud<pcl::PointNormal> mls_points;
    resampling(cloud_filtered, mls_points);
    //pcl::io::savePCDFile ("bun0-mls.pcd", mls_points);

    //visualization(cloud_filtered);
    //normalsVis(cloud_filtered, normals);

    //range_image(cloud_filtered);

    return (0);
}


