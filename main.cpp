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
#include "ground_segmentation.h"

int main ()
{

    std::string args = "b";
    if (args == "a"){
        //ground segmentation
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

        pcl::io::loadPCDFile ("/mrtstorage/users/chli/real_data/test_data3/1571220356.36.pcd", *cloud);
        ground_segmentation(cloud,cloud_filtered);
        visualization(cloud);
        visualization(cloud_filtered);
    }
    else {
    //ground segmentation


    preprocessing();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered3 (new pcl::PointCloud<pcl::PointXYZ>);
    // load the cloud data
    pcl::io::loadPCDFile ("/home/chli/CLionProjects/masterarbeit/data/0356_3_road_new2.pcd", *cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile ("/home/chli/CLionProjects/masterarbeit/data/pseudo_data/slope_10_ele_5.pcd", *cloud2);
    //home/chli/cc_code2/deeplab/kitti_image/testing/pcd/um_000085.pcd

    bspline_fitting(cloud2);

    visualization(cloud2);
    //radius_filter(cloud, cloud_filtered);
    //visualization(cloud_filtered);
    /*
    pcl::PointXYZ min;//用于存放三个轴的最小值
    pcl::PointXYZ max;//用于存放三个轴的最大值
    pcl::getMinMax3D(*cloud,min,max);

    std::cout<<"min.z = "<<min.x<<"\n"<<std::endl;
    std::cout<<"max.z = "<<max.x<<"\n"<<std::endl;
     */



    //std::cout << cloud -> points[0].z;
    //std::cout << cloud_filtered -> points[0].z;

    /* //usual normal estimation
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    normal_estimation(cloud_filtered, normals, cloud_with_normals );
    */
    //visualization(cloud_filtered);

    pcl::PointXYZ min;//用于存放三个轴的最小值
    pcl::PointXYZ max;//用于存放三个轴的最大值
    pcl::getMinMax3D(*cloud_filtered,min,max);

    std::cout<<"min.x = "<<min.x<<"\n"<<std::endl;
    std::cout<<"max.x = "<<max.x<<"\n"<<std::endl;

    voxel_grid(cloud_filtered,cloud_filtered3);

    pcl::getMinMax3D(*cloud_filtered3,min,max);

    std::cout<<"min.x = "<<min.x<<"\n"<<std::endl;
    std::cout<<"max.x = "<<max.x<<"\n"<<std::endl;



    std::vector<float> vect = road_feature(cloud_filtered3) ;
    std::cout<< "slope=" << *vect.begin()<< std::endl;
    std::cout<< "superelevation=" << *(vect.begin()+1)<< std::endl;

    //visualization(cloud_filtered3);


    float rough_slope = slope(cloud_filtered3);
    std::cout << "roght_slope ="<< rough_slope << std::endl;



    // resamling filter
    pcl::PointCloud<pcl::PointNormal>::Ptr mls_points (new pcl::PointCloud<pcl::PointNormal>);
    resampling(cloud_filtered3, mls_points);
    //pcl::io::savePCDFile ("bun0-mls.pcd", mls_points);


    //pcl::PCDWriter writer;
    //writer.write<pcl::PointXYZ>("/home/chli/CLionProjects/masterarbeit/data/0356_3_road_befor_voxfilter.pcd", *cloud_filtered, false);
    //writer.write<pcl::PointXYZ>("/home/chli/CLionProjects/masterarbeit/data/0356_3_road_after_voxfilter.pcd", *cloud_filtered, false);

    // greedy triangularation
    pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh);
    greedy_triangulation(mls_points,triangles);
    //pcl::io::saveVTKFile ("/home/cc/cc_code/masterarbeit/data/0356_3_road.vtk", *triangles);
    //range_image2(cloud_filtered);






    //marchingcubes reconstruction
    //marchingcubes (cloud_with_normals);

    //visualization(cloud_filtered);
    //normalsVis(cloud_filtered, normals);

    //range_image(cloud_filtered);
    }
    return (0);
}


