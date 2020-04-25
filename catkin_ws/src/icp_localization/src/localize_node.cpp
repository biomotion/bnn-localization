#include <iostream>
#include<ros/ros.h>
#include"ICPManager.hpp"

int main(int argc, char** argv){
    ros::init(argc, argv, "test_node");
    ros::NodeHandle n;
    ICPManager estimator(n, "/bags/map.pcd");

    ros::spin();
}