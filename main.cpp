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

//pcl::visualization::PCLVisualizer::Ptr
void rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    //return (viewer);
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        //boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}

int main ()
{


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile ("/home/chli/CLionProjects/masterarbeit/data/temp/1.pcd", *cloud2);
    //home/chli/cc_code2/deeplab/kitti_image/testing/pcd/um_000085.pcd
    ///home/chli/cc_code2/c++/surface/samples1.pcd




    rgbVis(cloud2);
    //bspline_fitting(cloud2);

    //visualization(cloud2);
    /*
    std::vector<float> vect = road_feature(cloud2) ;
    std::cout<< "slope=" << *vect.begin()<< std::endl;
    std::cout<< "superelevation=" << *(vect.begin()+1)<< std::endl;

    //visualization(cloud_filtered3);

    float rough_slope = slope(cloud2);
    std::cout << "roght_slope ="<< rough_slope << std::endl;

    */


    return (0);
}


