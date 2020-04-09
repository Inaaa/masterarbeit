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



#ifndef MASTERARBEIT_SURFACE_H
#define MASTERARBEIT_SURFACE_H

pcl::PolygonMesh greedy_triangulation(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals);

#endif //MASTERARBEIT_SURFACE_H
