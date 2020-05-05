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

#include"ICPManager.hpp"
using namespace std;
int main(int argc, char** argv){
    ros::init(argc, argv, "serial_icp_node");
    ros::NodeHandle n("~");
    ros::Publisher pub_pose = n.advertise<geometry_msgs::PoseStamped>("/car_pose", 1);
    ros::Publisher pub_points = n.advertise<sensor_msgs::PointCloud2>("/points_transformed", 1);
    ros::Publisher pub_imu = n.advertise<sensor_msgs::Imu>("/imu/data", 1);
    rosbag::Bag bag;
    rosbag::View view;
    std::string bag_file, map_file, car_frame, lidar_frame;
    double max_dist=10, tf_epsilon=1e-10, fit_epsilon=0.001, error_thres;
    int max_iter=100;

    n.param<std::string>("map_file", map_file, "/bags/itri/map.pcd");
    ICPManager manager(map_file.c_str());
    
    n.param<std::string>("bag_file", bag_file, "/bags/itri/ITRI_Public.bag");
    n.param<int>("max_iter", max_iter, (int)100);
    n.param<double>("max_distance", max_dist, 10);
    n.param<double>("transform_epsilon", tf_epsilon, 1e-10);
    n.param<double>("fitness_epsilon", fit_epsilon, 0.001);
    n.param<double>("error_thres", error_thres, 1.0);
    n.param<std::string>("car_frame_id", car_frame, "base_link");
    n.param<std::string>("lidar_frame_id", lidar_frame, "velodyne");
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
        listener.waitForTransform(car_frame, lidar_frame, ros::Time(0), ros::Duration(1.0));
        listener.lookupTransform(car_frame, lidar_frame,  
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
        static Eigen::Vector3d staged_gps = Eigen::Vector3d::Zero();
        cout << "topic|type: " << msg.getTopic() << "|" << msg.getDataType() << endl;

        tf2_msgs::TFMessage::ConstPtr tfs = msg.instantiate<tf2_msgs::TFMessage>();
        if(tfs != nullptr){
            cout << *tfs << endl;
        }
        

        sensor_msgs::Imu::ConstPtr imu = msg.instantiate<sensor_msgs::Imu>();
        if(imu != nullptr){
            pub_imu.publish(imu);
            if(!pose.topLeftCorner(3, 3).isApprox(Eigen::Matrix3d::Identity())) continue;
            Eigen::Quaterniond imu_orient(imu->orientation.w,
                                    imu->orientation.x,
                                    imu->orientation.y,
                                    imu->orientation.z);
            // manager.guessOrientation(imu_orient);
            // cout << "guess imu: \n" << imu_orient.toRotationMatrix() << endl;
        }


        
        geometry_msgs::PointStamped::ConstPtr gps = msg.instantiate<geometry_msgs::PointStamped>();;
        if(gps != nullptr){
            if(!pose.topRightCorner(3, 1).isApprox(Eigen::Vector3d::Zero())) continue;
            Eigen::Vector3d trans(gps->point.x,
                                gps->point.y,
                                gps->point.z);
            // manager.guessPosition(trans);
            // cout << "guess gps: \n" << trans << endl;
            staged_gps = trans;
        }
        
        //handle point cloud topic
        sensor_msgs::PointCloud2::ConstPtr pc = msg.instantiate<sensor_msgs::PointCloud2>();
        if(pc != nullptr){
            if(staged_gps.isApprox(Eigen::Vector3d::Zero())) continue;
            sensor_msgs::PointCloud2 pc_on_base, pc_out;
            PointCloud<PointXYZ>::Ptr input_cloud(new PointCloud<PointXYZ>);

            pcl_ros::transformPointCloud(eig_tf_base2lidar.cast<float>(), *pc, pc_on_base);
            pcl::fromROSMsg(pc_on_base, *input_cloud);
            cout << input_cloud->width << endl;
            if(pose.isApprox(Eigen::Matrix4d::Identity())){
                // First Aligning
                double min_score = 1e200;
                Eigen::Matrix3d try_orient=Eigen::Matrix3d::Identity(), best_orient;
                Eigen::Vector3d best_gps;
                manager.setParams(2, 1e-10, 1e-5, 20);
                for(int i=1; i<37; i++){
                    cout << "trying #" << i << endl;
                    manager.guessOrientation(try_orient);
                    manager.guessPosition(staged_gps);
                    manager.feedPC(input_cloud);
                    if(manager.getLastScore(error_thres)<min_score){
                        min_score = manager.getLastScore(error_thres);
                        best_orient = manager.getPose().topLeftCorner(3, 3);
                        best_gps = manager.getPose().topRightCorner(3, 1);
                        ROS_WARN("Better score");
                    }
                    // rotate for next try
                    try_orient = Eigen::AngleAxisd(M_PI/18, Eigen::Vector3d::UnitZ()) * try_orient;

                    pose = manager.getPose();
                    pcl_ros::transformPointCloud(pose.cast<float>(), pc_on_base, pc_out);
                    pc_out.header.frame_id = "map";
                    pub_points.publish(pc_out);
                }
                manager.guessOrientation(best_orient);
                manager.guessPosition(best_gps);

            }

            manager.setParams(max_dist, tf_epsilon, fit_epsilon, max_iter);
            manager.feedPC(input_cloud);
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