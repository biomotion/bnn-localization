#include <iostream>
#include<ros/ros.h>
#include<rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>


#include"ICPManager.hpp"
// Eigen::Matrix4f tf = Eigen::Matrix4f::Identity();
using namespace std;
int main(int argc, char** argv){
    ros::init(argc, argv, "serial_icp_node");
    ros::NodeHandle n;
    ros::Publisher pub_pose = n.advertise<geometry_msgs::PoseStamped>("/car_pose", 1);

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
        static Eigen::Matrix4f tf = Eigen::Matrix4f::Identity();
        cout << "topic|type: " << msg.getTopic() << "|" << msg.getDataType() << endl;
        // cout << msg.getDataType() << endl;
        // cout << tf << endl;

        if(tf.topLeftCorner(3, 3).isApprox(Eigen::Matrix3f::Identity())){
            sensor_msgs::Imu::ConstPtr imu = msg.instantiate<sensor_msgs::Imu>();
            if(imu != nullptr){
                Eigen::Quaternionf rot(imu->orientation.w,
                                        imu->orientation.x,
                                        imu->orientation.y,
                                        imu->orientation.z);
                tf.topLeftCorner(3, 3) = rot.toRotationMatrix();
                manager.setGuess(tf);
                cout << "guess rotation\n" << tf.topLeftCorner(3, 3) << endl;
                
            }
        }
        if(tf.topRightCorner(3, 1).isApprox(Eigen::Vector3f::Zero())){
            geometry_msgs::PointStamped::ConstPtr gps = msg.instantiate<geometry_msgs::PointStamped>();;
            if(gps != nullptr){
                tf.topRightCorner(3, 1) << gps->point.x,
                                            gps->point.y,
                                            gps->point.z;
                manager.setGuess(tf);
                cout << "guess translation\n" << tf.topRightCorner(3, 1) << endl;
            }
        }
        
        //handle point cloud topic
        sensor_msgs::PointCloud2::ConstPtr pc = msg.instantiate<sensor_msgs::PointCloud2>();
        if(pc != nullptr){
            pcl::PointCloud<pcl::PointXYZ>* input_cloud = new PointCloud<pcl::PointXYZ>;
            pcl::fromROSMsg(*pc, *input_cloud);


            manager.feedPC(*input_cloud);
            tf = manager.getPose();

            geometry_msgs::PoseStamped pose;
            Eigen::Matrix3f rot_matrix = tf.topLeftCorner(3, 3);
            Eigen::Quaternionf rot(rot_matrix);
            Eigen::Vector3f trans = tf.topRightCorner(3, 1);
            pose.header.frame_id = "map";
            pose.header.stamp = pc->header.stamp;
            pose.pose.orientation.w = rot.w();
            pose.pose.orientation.x = rot.x();
            pose.pose.orientation.y = rot.y();
            pose.pose.orientation.z = rot.z();
            pose.pose.position.x = trans.x();
            pose.pose.position.y = trans.y();
            pose.pose.position.z = trans.z();
            pub_pose.publish(pose);

        }


    }

    return EXIT_SUCCESS;
    
    
}