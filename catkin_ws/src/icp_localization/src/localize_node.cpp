#include <iostream>
#include<ros/ros.h>
#include"icp.hpp"

int main(int argc, char** argv){
    ros::init(argc, argv, "test_node");
    ros::NodeHandle n;
    ICPEstimator estimator(n, "/bags/map.pcd");

    ros::spin();
}