// PCL lib Functions for processing point clouds 

#include "processPointCloudsCustom.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr cloud_region(new pcl::PointCloud<PointT>);

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    ///voxel grid point reduction
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes,filterRes,filterRes);
    sor.filter (*cloud_filtered);

    ///region based filtering

    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloud_filtered);
    region.filter(*cloud_region);

    ///remove car roof points
    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5,-1.7,-1,1));
    roof.setMax(Eigen::Vector4f(2.6,1.7,-0.4,1));
    roof.setInputCloud(cloud_region);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for (int index:indices)
        inliers->indices.push_back(index);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud_region);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*cloud_region);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_region;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::ExtractIndices<PointT> extract;
    typename pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT>),cloud_obst (new  pcl::PointCloud<PointT>);

   extract.setInputCloud (cloud);
   extract.setIndices (inliers);
   extract.setNegative (false);
   extract.filter (*cloud_plane);

   std::cerr << "PointCloud representing the planar component: " << cloud_plane->width * cloud_plane->height << " data points." << std::endl;

   // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_obst);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult( cloud_plane, cloud_obst);
    return segResult;
}

template<typename PointT>
void ProcessPointClouds<PointT>::crossProduct(double vect_A[3], double vect_B[3], double (& cross_P)[3]){

    cross_P[0] = vect_A[1] * vect_B[2] - vect_A[2] * vect_B[1];
    cross_P[1] = vect_A[2] * vect_B[0] - vect_A[0] * vect_B[2];
    cross_P[2] = vect_A[0] * vect_B[1] - vect_A[1] * vect_B[0];
}

template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::Ransac_alg(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold){

    std::mt19937 generator(int(std::time(0)));
    std::uniform_int_distribution<int> dist(0, cloud->points.size() - 1);
    std::unordered_set<int> samples;
    float x1,y1, z1,x2,y2,z2,x3,y3,z3,x4,y4,z4, A, B, C, D;
    float distance ;


    std::unordered_set<int> inliersResult;


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

            if (distance < distanceThreshold){
                samples.insert(j);
            }


        }

        if(inliersResult.size()<samples.size()){
            inliersResult = samples;
        }

        return inliersResult;

    }
}

template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper( int indice, const std::vector<std::vector<float>>& points, std::vector<int> & cluster, std::vector<bool> &processed, KdTree* tree, float distanceTol){

    processed[indice] = true;
    cluster.push_back(indice);

    std::vector<int> nearest = tree->search(points[indice],distanceTol);

    for( int id:nearest){

        if(!processed[id]){
            clusterHelper(id,points,cluster,processed,tree,distanceTol);
        }
    }

}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>>points, KdTree* tree, float distanceTol)
{

    // TODO: Fill out this function to return list of indices for each cluster

    std::vector<std::vector<int>> clusters;

    std::vector<bool> processed(points.size(),false);

    int i =0;
    while(i<points.size()){
        if(processed[i]){
            i++;
            continue;
        }

        std::vector<int> cluster;
        clusterHelper(i, points, cluster, processed, tree, distanceTol);
        clusters.push_back(cluster);
        i++;
    }

    return clusters;

}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());


    std::unordered_set<int> inliers_set = Ransac_alg(cloud,maxIterations,distanceThreshold);

    for (auto i : inliers_set) {
        inliers->indices.push_back(i);
    }


    if (inliers->indices.size () == 0){
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    std::vector<std::vector<float>> points;

    KdTree* tree = new KdTree;

    for (int i=0; i<cloud->points.size(); i++)
    {

        points.push_back({cloud->points[i].x,cloud->points[i].y,cloud->points[i].z});
        tree->insert({cloud->points[i].x,cloud->points[i].y,cloud->points[i].z}, i);
    }

//    std::vector<std::vector<int>> ind_clusters = euclideanCluster(points, tree, 3.0);
    std::vector<std::vector<int>> ind_clusters = euclideanCluster(points, tree, clusterTolerance);

    int clusterId = 0;
    for(std::vector<int> cluster : ind_clusters)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for(int& indice: cluster) {
            cloud_cluster->push_back((*cloud)[indice]);
        }
        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        clusters.push_back(cloud_cluster);
        ++clusterId;
    }

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}