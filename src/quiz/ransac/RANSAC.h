//
// Created by robotics on 3/21/25.
//

#ifndef UDACITY_LIDAR_RANSAC_H
#define UDACITY_LIDAR_RANSAC_H

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>

#include "../../render/render.h"
#include <unordered_set>
//#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
//#include "../../processPointClouds.cpp"
#include <random>
#include <iostream>
#include <cmath>


template<typename PointT>
class RANSAC {
public:
    RANSAC();
    RANSAC(typename pcl::PointCloud<PointT>::Ptr cloud, int&  maxIterations, float& distanceTol);
    ~RANSAC();
    std::unordered_set<int> Ransac_alg();

private:
    void crossProduct();
    double cross_P_[3] = {0,0,0};
    double vect_A_[3] ;
    double vect_B_[3] ;
    int maxIterations_;
    float distanceTol_;
    typename pcl::PointCloud<PointT>::Ptr cloud_;
    std::unordered_set<int> inliersResult_;

};


#endif //UDACITY_LIDAR_RANSAC_H
