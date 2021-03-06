#include<pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <iostream>

using namespace pcl;

#ifndef ICP_MANAGER_H
#define ICP_MANAGER_H
class ICPManager{
private:
    Eigen::Matrix4Xd pose, guess;
    // PCL stuffs
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ, double> icp;
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud;
    void selectMapRange(float, float, float, float, float, float, pcl::PointCloud<PointXYZ>::Ptr&);
    void pointsPreCompute(PointCloud<PointXYZ>::Ptr, PointCloud<PointXYZ>::Ptr);
    void groundFilter(float zmin, float zmax, PointCloud<PointXYZ>::Ptr, PointCloud<PointXYZ>::Ptr);
    public:
    ICPManager();
    ICPManager(char const * map_file);
    void loadMap(std::string);
    void feedPC(pcl::PointCloud<pcl::PointXYZ>::Ptr&);
    // void feedPC(pcl::PointCloud<pcl::PointXYZ>, Eigen::Matrix4d); //feed point cloud with guess
    void guessTF(Eigen::Matrix4d g) { this->guess = g; }
    void guessOrientation(Eigen::Quaterniond q) { guessOrientation(q.toRotationMatrix()); } // guess orientation by quaternion
    void guessOrientation(Eigen::Matrix3d orien){ // guess orientation by rotation matrix
        if(this->guess.topRightCorner(3, 1).isApprox(Eigen::Vector3d::Zero()))
            this->guess = this->pose;
        this->guess.topLeftCorner(3, 3) = orien;
        // std::cout << "guess orientation\n" << orien << std::endl;
    }
    void guessPosition(Eigen::Vector3d posi){
        if(this->guess.topLeftCorner(3, 3).isApprox(Eigen::Matrix3d::Identity()))
            this->guess = this->pose;
        this->guess.topRightCorner(3, 1) = posi;
        // std::cout << "guess position\n" << posi << std::endl;
    }
    Eigen::Matrix4d getPose() { return this->pose; }
    void setParams(double max_d, double tf_esln, double fit_esln, uint16_t max_iter){
        this->icp.setMaxCorrespondenceDistance(max_d);
        this->icp.setTransformationEpsilon(tf_esln);
        this->icp.setEuclideanFitnessEpsilon(fit_esln);
        this->icp.setMaximumIterations(max_iter);
    }
    float getLastScore(float max){ return icp.getFitnessScore(max); }
};

#endif