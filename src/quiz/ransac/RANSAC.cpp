//
// Created by robotics on 3/21/25.
//

#include "RANSAC.h"

template<typename PointT>
RANSAC<PointT>::RANSAC(){

}

template<typename PointT>
RANSAC<PointT>::RANSAC(typename pcl::PointCloud<PointT>::Ptr cloud, int& maxIterations, float& distanceTol):cloud_(cloud),maxIterations_(maxIterations),distanceTol_(distanceTol){

}

template<typename PointT>
RANSAC<PointT>::~RANSAC(){
}

template<typename PointT>
std::unordered_set<int> RANSAC<PointT>::Ransac_alg(){

    std::mt19937 generator(int(std::time(0)));
    std::uniform_int_distribution<int> dist(0, cloud_->points.size() - 1);
    std::unordered_set<int> samples;
    float x1,y1, z1,x2,y2,z2,x3,y3,z3,x4,y4,z4, A, B, C, D;
    float distance ;


    for (int i=0; i<maxIterations_; i++){

        // use set as the container for samples because the 2 points need to be unique and set won't allow repeating the same sample twice.
        while( samples.size() < 3 ){
            samples.insert(dist(generator));
        }

        auto itt = samples.begin();
        x1 = cloud_->points[*itt].x;
        y1 = cloud_->points[*itt].y;
        z1 = cloud_->points[*itt].z;
        itt++;
        x2 = cloud_->points[*itt].x;
        y2 = cloud_->points[*itt].y;
        z2 = cloud_->points[*itt].z;
        itt++;
        x3 = cloud_->points[*itt].x;
        y3 = cloud_->points[*itt].y;
        z3 = cloud_->points[*itt].z;

         vect_A_[0] = (x2-x1);
         vect_A_[1] = (y2-y1);
         vect_A_[2] = (z2-z1);
         vect_B_[0] = (x3-x1);
         vect_B_[1] = (y3-y1);
         vect_B_[2] = (z3-z1);

        crossProduct();
        double norm_Cross_P = sqrt( pow(cross_P_[0],2) + pow(cross_P_[1],2)+ pow(cross_P_[2],2) );

        A = cross_P_[0]/norm_Cross_P;
        B = cross_P_[1]/norm_Cross_P;
        C = cross_P_[2]/norm_Cross_P;

        D = -( (A*x1) + (B*y1) + (C*z1));

        for (int j=0; j<cloud_->points.size(); j++){

            if (samples.count(j)>0){
                continue;
            }

            x3 = cloud_->points[j].x;
            y3 = cloud_->points[j].y;
            z3 = cloud_->points[j].z;

            distance = fabs( (A*x3)+(B*y3)+(C*z3)+D) / sqrt( pow(A,2) + pow(B,2) + pow(C,2) );

            if (distance < distanceTol_){
                samples.insert(j);
            }


        }

        if(inliersResult_.size()<samples.size()){
            inliersResult_ = samples;
        }

        return inliersResult_;

    }
}

template<typename PointT>
void RANSAC<PointT>::crossProduct(){

    cross_P_[0] = vect_A_[1] * vect_B_[2] - vect_A_[2] * vect_B_[1];
    cross_P_[1] = vect_A_[2] * vect_B_[0] - vect_A_[0] * vect_B_[2];
    cross_P_[2] = vect_A_[0] * vect_B_[1] - vect_A_[1] * vect_B_[0];
}
