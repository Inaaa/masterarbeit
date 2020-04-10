//
// Created by chli on 09.04.20.
//

//
// Created by chli on 09.04.20.
//
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include "surface.h"
#include <pcl/surface/marching_cubes_hoppe.h>// 移动立方体算法
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/visualization/pcl_visualizer.h>//可视化
#include <boost/thread/thread.hpp>//多线程

#include <string.h>
#include <string>

void greedy_triangulation(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals,
        pcl::PolygonMesh::Ptr triangles)
{
    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (cloud_with_normals);
    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;



    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius (500);

    // Set typical values for the parameters
    gp3.setMu (100);
    gp3.setMaximumNearestNeighbors (1570);
    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    gp3.setMinimumAngle(M_PI/18); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(false);

    // Get result
    gp3.setInputCloud (cloud_with_normals);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (*triangles);
    // Additional vertex information
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();

    // 显示结果图
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0); //设置背景
    viewer->addPolygonMesh(*triangles,"my"); //设置显示的网格
    //viewer->addCoordinateSystem (1.0); //设置坐标系
    viewer->initCameraParameters ();
    while (!viewer->wasStopped ()){
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }


}

/*
三维重构之移动立方体算法
*/
void marchingcubes (pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals)
{

    //创建搜索树
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (cloud_with_normals);

    //初始化 移动立方体算法 MarchingCubes对象，并设置参数
    pcl::MarchingCubes<pcl::PointNormal> *mc;
    mc = new pcl::MarchingCubesHoppe<pcl::PointNormal> ();
    /*
  if (hoppe_or_rbf == 0)
    mc = new pcl::MarchingCubesHoppe<pcl::PointNormal> ();
  else
  {
    mc = new pcl::MarchingCubesRBF<pcl::PointNormal> ();
    (reinterpret_cast<pcl::MarchingCubesRBF<pcl::PointNormal>*> (mc))->setOffSurfaceDisplacement (off_surface_displacement);
  }
    */

    //创建多变形网格，用于存储结果
    pcl::PolygonMesh mesh;

    //设置MarchingCubes对象的参数
    mc->setIsoLevel (0.0f);
    mc->setGridResolution (100, 100, 10);
    mc->setPercentageExtendGrid (0.0f);

    //设置搜索方法
    mc->setInputCloud (cloud_with_normals);

    //执行重构，结果保存在mesh中
    mc->reconstruct (mesh);

    //保存网格图
    pcl::io::savePLYFile("/mrtstorage/users/students/chli/real_data/test_data2/result2.ply", mesh);

    // 显示结果图
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0); //设置背景
    viewer->addPolygonMesh(mesh,"my"); //设置显示的网格
    //viewer->addCoordinateSystem (1.0); //设置坐标系
    viewer->initCameraParameters ();
    while (!viewer->wasStopped ()){
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

}