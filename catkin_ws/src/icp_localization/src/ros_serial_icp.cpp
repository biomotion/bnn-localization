#include <iostream>
#include<ros/ros.h>
#include<rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h>
#include <tf2_msgs/TFMessage.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl_ros/transforms.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include"ICPManager.hpp"
using namespace std;
int main(int argc, char** argv){
    ros::init(argc, argv, "serial_icp_node");
    ros::NodeHandle n("~");
    ros::Publisher pub_pose = n.advertise<geometry_msgs::PoseStamped>("/car_pose", 1);
    ros::Publisher pub_points = n.advertise<sensor_msgs::PointCloud2>("/points_transformed", 1);
    rosbag::Bag bag;
    rosbag::View view;
    std::string bag_file, map_file;
    double max_dist=10, tf_epsilon=1e-10, fit_epsilon=0.001;
    int max_iter=100;

    n.param<std::string>("map_file", map_file, "/bags/itri/map.pcd");
    ICPManager manager(map_file.c_str());
    
    n.param<std::string>("bag_file", bag_file, "/bags/itri/ITRI_Public.bag");
    n.param<int>("max_iter", max_iter, (int)100);
    n.param<double>("max_distance", max_dist, 10);
    n.param<double>("transform_epsilon", tf_epsilon, 1e-10);
    n.param<double>("fitness_epsilon", fit_epsilon, 0.001);
    // cout << max_iter << endl
    //     << max_dist << endl
    //     << tf_epsilon << endl
    //     << fit_epsilon << endl;
    manager.setParams(max_dist, tf_epsilon, fit_epsilon, max_iter);
    bag.open(bag_file, rosbag::bagmode::Read);
    view.addQuery(bag);

    tf::TransformListener listener;
    tf::StampedTransform t_base2lidar;
    Eigen::Matrix4f eig_tf_base2lidar = Eigen::Matrix4f::Identity();
    try{
        listener.waitForTransform("/base_link", "/velodyne", ros::Time(0), ros::Duration(1.0));
        listener.lookupTransform("/base_link", "/velodyne",  
                               ros::Time(0), t_base2lidar);
        Eigen::Quaterniond r;
        Eigen::Vector3d t;
        tf::quaternionTFToEigen(t_base2lidar.getRotation(), r);
        eig_tf_base2lidar.topLeftCorner(3, 3) = r.cast<float>().toRotationMatrix();
        tf::vectorTFToEigen(t_base2lidar.getOrigin(), t);
        eig_tf_base2lidar.topRightCorner(3, 1) = t.cast<float>();
        // cout << r.toRotationMatrix() << endl;
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }

    for (const rosbag::MessageInstance& msg : view){
        if(!ros::ok()) break;
        static Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        static Eigen::Quaternionf staged_imu = Eigen::Quaternionf::Identity();

        cout << "topic|type: " << msg.getTopic() << "|" << msg.getDataType() << endl;

        tf2_msgs::TFMessage::ConstPtr tfs = msg.instantiate<tf2_msgs::TFMessage>();
        if(tfs != nullptr){
            cout << *tfs << endl;
        }
        

        sensor_msgs::Imu::ConstPtr imu = msg.instantiate<sensor_msgs::Imu>();
        if(imu != nullptr){
            if(!pose.topLeftCorner(3, 3).isApprox(Eigen::Matrix3f::Identity())) continue;
            Eigen::Quaternionf imu_orient(imu->orientation.w,
                                    imu->orientation.x,
                                    imu->orientation.y,
                                    imu->orientation.z);
            Eigen::Quaterniond t;
            tf::quaternionTFToEigen(t_base2lidar.getRotation(), t);
            manager.guessOrientation(t.cast<float>() * imu_orient);
        }


        
        geometry_msgs::PointStamped::ConstPtr gps = msg.instantiate<geometry_msgs::PointStamped>();;
        if(gps != nullptr){
            if(!pose.topRightCorner(3, 1).isApprox(Eigen::Vector3f::Zero())) continue;
            Eigen::Vector3f trans(gps->point.x,
                                gps->point.y,
                                gps->point.z);
            manager.guessPosition(trans);
        }
        
        //handle point cloud topic
        sensor_msgs::PointCloud2::ConstPtr pc = msg.instantiate<sensor_msgs::PointCloud2>();
        if(pc != nullptr){
            pcl::PointCloud<pcl::PointXYZ>* input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new PointCloud<PointXYZ>), cloud2(new PointCloud<PointXYZ>);
            pcl::VoxelGrid<pcl::PointXYZ> grid_filter;
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> noise_filter;
            pcl::fromROSMsg(*pc, *input_cloud);
            sensor_msgs::PointCloud2 pc_out;


            grid_filter.setInputCloud(PointCloud<PointXYZ>::Ptr(input_cloud));
            grid_filter.setLeafSize(0.2f, 0.2f, 0.2f);
            grid_filter.filter(*cloud2);
            noise_filter.setInputCloud(cloud2);
            noise_filter.setMeanK(32);
            noise_filter.setStddevMulThresh(1.0);
            noise_filter.filter(*filtered_cloud);

            manager.feedPC(filtered_cloud);

            // manager.feedPC(*input_cloud);
            pose = manager.getPose();
            pcl_ros::transformPointCloud(pose, *pc, pc_out);
            pc_out.header.frame_id = "map";
            pub_points.publish(pc_out);
            // cout << "tf:\n" << eig_tf_base2lidar.inverse() << endl;
            // pose = eig_tf_base2lidar.inverse() * pose;
            pose.topLeftCorner(3, 3) = eig_tf_base2lidar.topLeftCorner(3, 3).inverse() * pose.topLeftCorner(3, 3);
            pose.topRightCorner(3, 1) = -eig_tf_base2lidar.topRightCorner(3, 1) + pose.topRightCorner(3, 1);
            cout << "after tf\n" << pose << endl;

            geometry_msgs::PoseStamped pose_msg;
            Eigen::Matrix3f rot_matrix = pose.topLeftCorner(3, 3);
            Eigen::Quaternionf rot(rot_matrix);
            Eigen::Vector3f trans = pose.topRightCorner(3, 1);
            pose_msg.header.frame_id = "map";
            pose_msg.header.stamp = pc->header.stamp;
            pose_msg.pose.orientation.w = rot.w();
            pose_msg.pose.orientation.x = rot.x();
            pose_msg.pose.orientation.y = rot.y();
            pose_msg.pose.orientation.z = rot.z();
            pose_msg.pose.position.x = trans.x();
            pose_msg.pose.position.y = trans.y();
            pose_msg.pose.position.z = trans.z();

            pub_pose.publish(pose_msg);

        }


    }

    return EXIT_SUCCESS;
    
    
}