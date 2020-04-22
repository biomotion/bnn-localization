#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include<pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <iostream>

using namespace pcl;
class ICPEstimator{
private:
    // ROS stuffs
    ros::NodeHandle n;
    ros::Subscriber sub_pc;
    ros::Publisher pub_pose;

    // PCL stuffs
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    pcl::PointCloud<pcl::PointXYZ>* map_cloud;
    void pc_callback(const sensor_msgs::PointCloud2ConstPtr& input_msg){
        pcl::PointCloud<pcl::PointXYZ>* input_cloud = new PointCloud<pcl::PointXYZ>;
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_ptr(input_cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr map_ptr(map_cloud);
        pcl::PointCloud<pcl::PointXYZ> final_cloud;
        pcl::fromROSMsg(*input_msg, *input_cloud);

        icp.setInputSource(input_ptr);
        icp.setInputTarget(map_ptr);

        icp.align(final_cloud);
        std::cout << "has converged: " << icp.hasConverged() <<std::endl;
        std::cout << "score: " <<icp.getFitnessScore() << std::endl; 
        std::cout << icp.getFinalTransformation() << std::endl;

    }
public:
    ICPEstimator();
    ICPEstimator(ros::NodeHandle nh, std::string map_file){
        this->n = nh;
        this->sub_pc = n.subscribe("/velodyne_points", 1, &ICPEstimator::pc_callback, this);
        this->pub_pose = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 1);
        this->map_cloud = new pcl::PointCloud<pcl::PointXYZ>;
        if(pcl::io::loadPCDFile (map_file, *map_cloud) < 0){
            ROS_ERROR("map file not found");
            PCL_ERROR("Couldn't read file %s\n", map_file);
        }
        this->icp.setMaxCorrespondenceDistance(100);
        this->icp.setTransformationEpsilon(1e-10);
        this->icp.setEuclideanFitnessEpsilon(0.001);
        this->icp.setMaximumIterations(100);
    }
};