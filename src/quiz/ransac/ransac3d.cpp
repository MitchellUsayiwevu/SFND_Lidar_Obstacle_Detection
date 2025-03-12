//
// Created by mitchell on 3/12/25.
//

// Quiz on implementing simple RANSAC plane fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <random>
#include <iostream>
#include <cmath>

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // Add inliers
    float scatter = 0.6;
    for(int i = -5; i < 5; i++)
    {
        double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
        double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
        pcl::PointXYZ point;
        point.x = i+scatter*rx;
        point.y = i+scatter*ry;
        point.z = 0;

        cloud->points.push_back(point);
    }
    // Add outliers
    int numOutliers = 10;
    while(numOutliers--)
    {
        double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
        double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
        pcl::PointXYZ point;
        point.x = 5*rx;
        point.y = 5*ry;
        point.z = 0;

        cloud->points.push_back(point);

    }
    cloud->width = cloud->points.size();
    cloud->height = 1;

    return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
    viewer->addCoordinateSystem (1.0);
    return viewer;
}

void crossProduct(double vect_A[3], double vect_B[3], double (& cross_P)[3])

{

    cross_P[0] = vect_A[1] * vect_B[2] - vect_A[2] * vect_B[1];
    cross_P[1] = vect_A[2] * vect_B[0] - vect_A[0] * vect_B[2];
    cross_P[2] = vect_A[0] * vect_B[1] - vect_A[1] * vect_B[0];
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult;
    std::vector<double>line_eqn;
    srand(time(NULL));

    std::mt19937 generator(int(std::time(0)));
    std::uniform_int_distribution<int> dist(0, cloud->points.size() - 1);
    std::unordered_set<int> samples;
    float x1,y1, z1,x2,y2,z2,x3,y3,z3,x4,y4,z4, A, B, C, D;
    float distance ;

    // TODO: Fill in this function

    for (int i=0; i<maxIterations; i++){

        // use set as the container for samples because the 2 points need to be unique and set won't allow repeating the same sample twice.
        while( samples.size() < 3 ){
            samples.insert(dist(generator));
        }

        auto itt = samples.begin();
        x1 = cloud->points[*itt].x;
        y1 = cloud->points[*itt].y;
        z1 = cloud->points[*itt].z;
        itt++;
        x2 = cloud->points[*itt].x;
        y2 = cloud->points[*itt].y;
        z2 = cloud->points[*itt].z;
        itt++;
        x3 = cloud->points[*itt].x;
        y3 = cloud->points[*itt].y;
        z3 = cloud->points[*itt].z;

        double vect_A[3] = { (x2-x1), (y2-y1),(z2-z1)};
        double vect_B[3] = { (x3-x1), (y3-y1),(z3-z1)};
        double cross_P[3];

        crossProduct( vect_A,  vect_B,  cross_P);
        double norm_Cross_P = sqrt( pow(cross_P[0],2) + pow(cross_P[1],2)+ pow(cross_P[2],2) );

        A = cross_P[0]/norm_Cross_P;
        B = cross_P[1]/norm_Cross_P;
        C = cross_P[2]/norm_Cross_P;

        D = -( (A*x1) + (B*y1) + (C*z1));

        for (int j=0; j<cloud->points.size(); j++){

            if (samples.count(j)>0){
                continue;
            }

            x3 = cloud->points[j].x;
            y3 = cloud->points[j].y;
            z3 = cloud->points[j].z;

            distance = fabs( (A*x3)+(B*y3)+(C*z3)+D) / sqrt( pow(A,2) + pow(B,2) + pow(C,2) );

            if (distance < distanceTol){
                samples.insert(j);
            }


        }

        if(inliersResult.size()<samples.size()){
            inliersResult = samples;
            line_eqn.clear();
            line_eqn.push_back(A);
            line_eqn.push_back(B);
            line_eqn.push_back(C);
            line_eqn.push_back(D);
        }

        return inliersResult;

    }


    // For max iterations

    // Randomly sample subset and fit line

    // Measure distance between every point and fitted line
    // If distance is smaller than threshold count it as inlier

    // Return indicies of inliers from fitted line with most inliers

    return inliersResult;

}

int main ()
{

    // Create viewer
    pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

    // Create data
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();


    // TODO: Change the max iteration and distance tolerance arguments for Ransac function
    std::unordered_set<int> inliers = Ransac(cloud, 100, 1.0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

    for(int index = 0; index < cloud->points.size(); index++)
    {
        pcl::PointXYZ point = cloud->points[index];
        if(inliers.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }


    // Render 2D point cloud with inliers and outliers
    if(inliers.size())
    {
        renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
        renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
    }
    else
    {
        renderPointCloud(viewer,cloud,"data");
    }

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    }

}

