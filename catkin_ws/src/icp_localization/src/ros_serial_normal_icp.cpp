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

#include"NormalICPManager.hpp"
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
    NormalICPManager manager(map_file.c_str());
    
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
    Eigen::Matrix4d eig_tf_base2lidar = Eigen::Matrix4d::Identity();
    try{
        listener.waitForTransform("/base_link", "/velodyne", ros::Time(0), ros::Duration(1.0));
        listener.lookupTransform("/base_link", "/velodyne",  
                               ros::Time(0), t_base2lidar);
        Eigen::Quaterniond r;
        Eigen::Vector3d t;
        tf::quaternionTFToEigen(t_base2lidar.getRotation(), r);
        eig_tf_base2lidar.topLeftCorner(3, 3) = r.toRotationMatrix();
        tf::vectorTFToEigen(t_base2lidar.getOrigin(), t);
        eig_tf_base2lidar.topRightCorner(3, 1) = t;
        // cout << r.toRotationMatrix() << endl;
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }

    for (const rosbag::MessageInstance& msg : view){
        if(!ros::ok()) break;
        static Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
        static Eigen::Quaterniond staged_imu = Eigen::Quaterniond::Identity();

        cout << "topic|type: " << msg.getTopic() << "|" << msg.getDataType() << endl;

        tf2_msgs::TFMessage::ConstPtr tfs = msg.instantiate<tf2_msgs::TFMessage>();
        if(tfs != nullptr){
            cout << *tfs << endl;
        }
        

        sensor_msgs::Imu::ConstPtr imu = msg.instantiate<sensor_msgs::Imu>();
        if(imu != nullptr){
            if(!pose.topLeftCorner(3, 3).isApprox(Eigen::Matrix3d::Identity())) continue;
            Eigen::Quaterniond imu_orient(imu->orientation.w,
                                    imu->orientation.x,
                                    imu->orientation.y,
                                    imu->orientation.z);
            // Eigen::Quaterniond t;
            // tf::quaternionTFToEigen(t_base2lidar.getRotation(), t);
            manager.guessOrientation(imu_orient);
            cout << "guess imu: \n" << imu_orient.toRotationMatrix() << endl;
        }


        
        geometry_msgs::PointStamped::ConstPtr gps = msg.instantiate<geometry_msgs::PointStamped>();;
        if(gps != nullptr){
            if(!pose.topRightCorner(3, 1).isApprox(Eigen::Vector3d::Zero())) continue;
            Eigen::Vector3d trans(gps->point.x,
                                gps->point.y,
                                gps->point.z);
            manager.guessPosition(trans);
            cout << "guess gps: \n" << trans << endl;
        }
        
        //handle point cloud topic
        sensor_msgs::PointCloud2::ConstPtr pc = msg.instantiate<sensor_msgs::PointCloud2>();
        if(pc != nullptr){
            sensor_msgs::PointCloud2 pc_on_base, pc_out;
            PointCloud<PointXYZ>::Ptr input_cloud(new PointCloud<PointXYZ>),
                                      cloud2(new PointCloud<PointXYZ>),
                                      cloud3(new PointCloud<PointXYZ>);
            pcl::VoxelGrid<pcl::PointXYZ> grid_filter;
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> noise_filter;

            pcl_ros::transformPointCloud(eig_tf_base2lidar.cast<float>(), *pc, pc_on_base);
            pcl::fromROSMsg(pc_on_base, *input_cloud);

            noise_filter.setInputCloud(input_cloud);
            noise_filter.setMeanK(64);
            noise_filter.setStddevMulThresh(1.0);
            noise_filter.filter(*cloud2);
            grid_filter.setInputCloud(cloud2);
            grid_filter.setLeafSize(0.1f, 0.1f, 0.1f);
            grid_filter.filter(*cloud3);


            manager.feedPC(cloud3);

            // manager.feedPC(*input_cloud);
            pose = manager.getPose();
            pcl_ros::transformPointCloud(pose.cast<float>(), pc_on_base, pc_out);
            pc_out.header.frame_id = "map";
            pub_points.publish(pc_out);

            geometry_msgs::PoseStamped pose_msg;
            Eigen::Matrix3d rot_matrix = pose.topLeftCorner(3, 3);
            Eigen::Quaterniond rot(rot_matrix);
            Eigen::Vector3d trans = pose.topRightCorner(3, 1);
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