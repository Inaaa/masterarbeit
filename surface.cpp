//
// Created by chli on 09.04.20.
//

//
// Created by chli on 09.04.20.
//
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
//#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/gp3.h>
#include "surface.h"


pcl::PolygonMesh greedy_triangulation(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals)
{
    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (cloud_with_normals);
    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;


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
    gp3.reconstruct (triangles);
    // Additional vertex information
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();

    return triangles;

}


