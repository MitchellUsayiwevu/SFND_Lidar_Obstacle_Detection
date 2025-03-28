/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
//#include "processPointClouds.h"
#include "processPointCloudsCustom.h"
// using templates for processPointClouds so also include .cpp to help linker
//#include "processPointClouds.cpp"
#include "processPointCloudsCustom.cpp"
#include <memory>
#include <iostream>


std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
//    bool renderScene = true;
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor
    double setGroundSlope{0};
    Lidar* lidar = new Lidar(cars, setGroundSlope );
    pcl::PointCloud<pcl::PointXYZ>::Ptr LidarScan = lidar->scan();   //scan the area to generate point cloud
//    renderRays(viewer, lidar->position,LidarScan); // visualize point cloud
//    renderPointCloud(viewer,LidarScan, "mypointclouddata",Color(1,0,0));

    // TODO:: Create point processor

    ProcessPointClouds<pcl::PointXYZ> myprocess;
//    ProcessPointClouds<pcl::PointXYZ>* myprocess2ptr =  new ProcessPointClouds<pcl::PointXYZ>();
    std::pair< pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> mysegments =  myprocess.SegmentPlane(LidarScan,1000,0.2);
    renderPointCloud(viewer,mysegments.first, "cloud_plane",Color(1,0,0));
//    renderPointCloud(viewer,mysegments.second, "cloud_obst",Color(0,1,0));

    std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr> myclusters =  myprocess.Clustering( mysegments.second,1.0, 3, 30);

    std::vector<Color> colors = {Color(1,1,0), Color(0,0,1),Color(0,1,1),Color(1,0,1)};

    int pointcloudId =0;
    bool render_box = true;

    for(auto& cluster : myclusters){
        std::cout<<"cluster size";
        myprocess.numPoints(cluster);
        renderPointCloud(viewer,cluster, "obstacle"+std::to_string(pointcloudId),colors[pointcloudId%colors.size()]);

        if (render_box){
            Box box = myprocess.BoundingBox(cluster);
            renderBox(viewer, box, pointcloudId,colors[pointcloudId%colors.size()]);
        }
        pointcloudId++;
    }


}
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)

//void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{

//    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
//    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.2, Eigen::Vector4f (-12, -6, -10, 1), Eigen::Vector4f ( 18, 9, 10, 1));
    renderPointCloud(viewer,filterCloud,"filterCloud");
//    renderPointCloud(viewer,inputCloud,"inputCloud");

    ///segmenting the ground plane from the obstacles
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr>  pointCloudSegments = pointProcessorI->SegmentPlane(filterCloud, 1000 , 0.2);
    renderPointCloud(viewer,pointCloudSegments.first, "plane_segment", Color(1,0,0));
//    renderPointCloud(viewer, pointCloudSegments.second,"obst_segment", Color(0,0,1) );

    /// clustering the obstacles
//    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> obst_segments = pointProcessorI->Clustering(pointCloudSegments.second, 0.53, 10, 500);
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> obst_segments = pointProcessorI->Clustering(pointCloudSegments.second, 0.4, 10, 500);

    std::vector<Color> colors = {Color(1,1,0), Color(0,0,1),Color(0,1,1),Color(1,0,1),Color( 0,1,0), Color(1,0,0)};
    int pointcloudId =0;
    bool render_box = true;

    for(auto& cluster : obst_segments){
        std::cout<<"cluster size";
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer,cluster, "obstacle"+std::to_string(pointcloudId),colors[pointcloudId%colors.size()]);

        if (render_box){
            Box box = pointProcessorI->BoundingBox(cluster);
            renderBox(viewer, box, pointcloudId,colors[pointcloudId%colors.size()]);
        }
        pointcloudId++;
    }
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
///    simpleHighway(viewer);

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

///    cityBlock(viewer);
///
///    while (!viewer->wasStopped ())
///    {
///        viewer->spinOnce ();
///    }

    while (!viewer->wasStopped ())
    {

        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
    }
}