#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include<iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>

using namespace std;

ros::Publisher pub_map;
sensor_msgs::PointCloud2 output;
void timer_cb(const ros::TimerEvent& event){
    static int id = 0;
    output.header.stamp = ros::Time(0);
    output.header.seq = id++;
    pub_map.publish(output);
}

int main (int argc, char *argv[])
{
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    // pcl::visualization::CloudViewer viewer("Cloud Viewer");

    ros::init(argc, argv, "read_file_node");
    ros::NodeHandle n;
    ros::Timer timer;
    std::string file_name;
    bool show_window = false;

    pub_map = n.advertise<sensor_msgs::PointCloud2>("/map", 1);

    if(!ros::param::get("~file", file_name))
        file_name = "/bags/map.pcd";

    if(!ros::param::get("show_window", show_window))
        show_window = false;

    
    if (pcl::io::loadPCDFile (file_name, *cloud) < 0) {
        PCL_ERROR("Couldn't read file %s\n", file_name);
        return -1;
    }else{
        pcl_conversions::fromPCL(*cloud, output);
        output.header.frame_id = "map";
        timer = n.createTimer(ros::Duration(1), timer_cb);
        ros::spin();
    }


    return 0;
}