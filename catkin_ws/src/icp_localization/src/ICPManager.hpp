#include<pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <iostream>

using namespace pcl;

#ifndef ICP_MANAGER_H
#define ICP_MANAGER_H
class ICPManager{
private:
    Eigen::Matrix4f pose, guess;
    // PCL stuffs
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud;
    void selectMapRange(float, float, float, float, float, float, pcl::PointCloud<PointXYZ>::Ptr&);
public:
    ICPManager();
    ICPManager(char const * map_file);
    void loadMap(std::string);
    void feedPC(pcl::PointCloud<pcl::PointXYZ>&);
    // void feedPC(pcl::PointCloud<pcl::PointXYZ>, Eigen::Matrix4d); //feed point cloud with guess
    void guessTF(Eigen::Matrix4f g) { this->guess = g; }
    void guessOrientation(Eigen::Quaternionf q) { guessOrientation(q.toRotationMatrix()); } // guess orientation by quaternion
    void guessOrientation(Eigen::Matrix3f orien){ // guess orientation by rotation matrix
        if(this->guess.topRightCorner(3, 1).isApprox(Eigen::Vector3f::Zero()))
            this->guess = this->pose;
        this->guess.topLeftCorner(3, 3) = orien;
        std::cout << "guess orientation\n" << orien << std::endl;
    }
    void guessPosition(Eigen::Vector3f posi){
        if(this->guess.topLeftCorner(3, 3).isApprox(Eigen::Matrix3f::Identity()))
            this->guess = this->pose;
        this->guess.topRightCorner(3, 1) = posi;
        std::cout << "guess position\n" << posi << std::endl;
    }
    Eigen::Matrix4f getPose() { return this->pose; }
    void setParams(double max_d, double tf_esln, double fit_esln, uint16_t max_iter){
        this->icp.setMaxCorrespondenceDistance(max_d);
        this->icp.setTransformationEpsilon(tf_esln);
        this->icp.setEuclideanFitnessEpsilon(fit_esln);
        this->icp.setMaximumIterations(max_iter);
    }

};

#endif