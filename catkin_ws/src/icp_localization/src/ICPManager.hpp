#include<pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <iostream>

using namespace pcl;

#ifndef ICP_MANAGER_H
#define ICP_MANAGER_H
class ICPManager{
private:
    Eigen::Matrix4f pose, guess;
    // PCL stuffs
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    pcl::PointCloud<pcl::PointXYZ>* map_cloud;
public:
    ICPManager();
    ICPManager(char const * map_file);
    void loadMap(std::string);
    void feedPC(pcl::PointCloud<pcl::PointXYZ>&);
    // void feedPC(pcl::PointCloud<pcl::PointXYZ>, Eigen::Matrix4d); //feed point cloud with guess
    void setGuess(Eigen::Matrix4f g) { this->guess = g; }
    Eigen::Matrix4f getPose() { return this->pose; }

};

#endif