//
// Created by chli on 10.04.20.
//

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/common.h>
#include "preprocessing.h"
#include "visualization.h"

void preprocessing()
{
    // Load input file into a PointCloud<T> with an appropriate type
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // Load bun0.pcd -- should be available with the PCL archive in test

    pcl::io::loadPCDFile("/home/chli/CLionProjects/masterarbeit/data/0.000000000.pcd", *cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>());

    for (int i = 0; i < cloud->points.size (); ++i)
    {
        if (cloud->points[i].x != 0)
        {
            cloud1->push_back(cloud->points[i]);

        }
        std::cout << cloud->points[i] << std::endl;
    }
    //std::cout << cloud->points[500] << std::endl;
    std::cout << cloud->points.size () << std::endl;
    visualization(cloud1);

    pcl::PointXYZ min;//用于存放三个轴的最小值
    pcl::PointXYZ max;//用于存放三个轴的最大值
    pcl::getMinMax3D(*cloud,min,max);


    std::cout<<"min.x = "<<min.x<<"\n"<<std::endl;
    std::cout<<"max.x = "<<max.x<<"\n"<<std::endl;


    //pcl::PCDWriter writer;
    //writer.write<pcl::PointXYZ>("/mrtstorage/users/chli/real_data/test_data3/1571220356.3.pcd", *cloud, false);
    //writer.write<pcl::PointXYZ>("/mrtstorage/users/chli/real_data/test_data3/1571220356.36.pcd", *cloud1);

}