#include <iostream>
#include<ros/ros.h>
#include<rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PointStamped.h>

#include <pcl/filters/voxel_grid.h>

#include"ICPManager.hpp"

using namespace std;
int main(int argc, char** argv){
    ros::init(argc, argv, "serial_icp_node");
    ros::NodeHandle n;

    ICPManager manager("/bags/itri/map.pcd");

    std::string bag_file;
    n.param<std::string>("bag_file", bag_file, "/bags/itri/ITRI_Public.bag");

    cout << bag_file << endl;
    rosbag::Bag bag;
    rosbag::View view;
    bag.open(bag_file, rosbag::bagmode::Read);
    view.addQuery(bag);
    // std::cout << bag.getSize() << std::endl;

    for (const rosbag::MessageInstance& msg : view){
        if(!ros::ok()) break;
        cout << msg.getTopic() << endl;

        static Eigen::Matrix4f tf = Eigen::Matrix4f::Identity();

        if(true){
            // ROS_WARN("Not initialized");
            sensor_msgs::Imu::ConstPtr imu = msg.instantiate<sensor_msgs::Imu>();
            if(imu != nullptr){
                Eigen::Quaternionf rot(imu->orientation.w,
                                        imu->orientation.x,
                                        imu->orientation.y,
                                        imu->orientation.z);
                tf.topLeftCorner(3, 3) = rot.toRotationMatrix();
                manager.setGuess(tf);
                cout << "guess\n" << tf << endl;
                
            }

            geometry_msgs::PointStamped::ConstPtr gps = msg.instantiate<geometry_msgs::PointStamped>();
            if(gps != nullptr){
                tf.topRightCorner(3, 1) << gps->point.x,
                                            gps->point.y,
                                            gps->point.z;
                manager.setGuess(tf);
                cout << "guess\n" << tf << endl;
                // ROS_INFO("GOT GPS");
            }
        }
        
        //handle point cloud topic
        sensor_msgs::PointCloud2::ConstPtr pc = msg.instantiate<sensor_msgs::PointCloud2>();
        if(pc != nullptr){
            pcl::PointCloud<pcl::PointXYZ>* input_cloud = new PointCloud<pcl::PointXYZ>;
            pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
            pcl::fromROSMsg(*pc, *input_cloud);

            pcl::VoxelGrid<pcl::PointXYZ> sor;
            sor.setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr(input_cloud));
            sor.setLeafSize (0.2f, 0.2f, 0.2f);
            sor.filter (cloud_filtered);

            manager.feedPC(cloud_filtered);
            tf = manager.getPose();
        }

        

    }

    return EXIT_SUCCESS;
    
    
}