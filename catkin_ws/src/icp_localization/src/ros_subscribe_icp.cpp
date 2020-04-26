#include <iostream>
#include<ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include"ICPManager.hpp"


void cb_pc(const sensor_msgs::PointCloud2::ConstPtr& msg){

}

int main(int argc, char** argv){
    ros::init(argc, argv, "subscribe_icp");
    ros::NodeHandle n;
    // ICPManager manager("/bags/map.pcd");

    ros::Subscriber sub_pc = n.subscribe("/velodyne_points", 10, cb_pc);

    ros::spin();
}