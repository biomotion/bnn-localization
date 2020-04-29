#include <iostream>
#include<ros/ros.h>
#include<rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_msgs/TFMessage.h>


#include"ICPManager.hpp"
// Eigen::Matrix4f tf = Eigen::Matrix4f::Identity();
using namespace std;
int main(int argc, char** argv){
    ros::init(argc, argv, "serial_icp_node");
    ros::NodeHandle n;
    ros::Publisher pub_pose = n.advertise<geometry_msgs::PoseStamped>("/car_pose", 1);
    rosbag::Bag bag;
    rosbag::View view;
    std::string bag_file;
    double max_dist=10, tf_epsilon=1e-10, fit_epsilon=0.001;
    int max_iter=100;

    ICPManager manager("/bags/itri/map.pcd");
    
    n.param<std::string>("bag_file", bag_file, "/bags/itri/ITRI_Public.bag");
    n.param<int>("max_iter", max_iter, (int)100);
    n.param<double>("max_distance", max_dist, 10);
    n.param<double>("transform_epsilon", tf_epsilon, 1e-10);
    n.param<double>("fitness_epsilon", fit_epsilon, 0.001);
    cout << max_iter << endl
        << max_dist << endl
        << tf_epsilon << endl
        << fit_epsilon << endl;
    manager.setParams(max_dist, tf_epsilon, fit_epsilon, max_iter);
    bag.open(bag_file, rosbag::bagmode::Read);
    view.addQuery(bag);

    for (const rosbag::MessageInstance& msg : view){
        if(!ros::ok()) break;
        static Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        static Eigen::Quaternionf staged_imu = Eigen::Quaternionf::Identity();

        cout << "topic|type: " << msg.getTopic() << "|" << msg.getDataType() << endl;


            tf2_msgs::TFMessage::ConstPtr tf_msg = msg.instantiate<tf2_msgs::TFMessage>();
            if(tf_msg != nullptr){
                for(geometry_msgs::TransformStamped tf : tf_msg->transforms ){
                    if(tf.header.frame_id=="base_link" && tf.child_frame_id=="velodyne"){
                        Eigen::Matrix3f rot_imu_to_lidar;
                        Eigen::Matrix3f changed_imu;
                        // cout << "update tf imu" << endl;
                        cout << "base_link to velodyne" << endl;
                        // cout << tf.transform << endl;
                        rot_imu_to_lidar = Eigen::Quaternionf(tf.transform.rotation.w,
                                                            tf.transform.rotation.x,
                                                            tf.transform.rotation.y,
                                                            tf.transform.rotation.z).toRotationMatrix();
                        // tf_imu_to_lidar.topRightCorner(3, 1) << tf.transform.translation.x,
                        //                                         tf.transform.translation.y,
                        //                                         tf.transform.translation.z;
                        changed_imu = rot_imu_to_lidar * staged_imu.toRotationMatrix();
                        manager.guessOrientation(changed_imu);

                    }
                }
            }

            sensor_msgs::Imu::ConstPtr imu = msg.instantiate<sensor_msgs::Imu>();
            if(imu != nullptr){
                // always ipdate imu orientation
                Eigen::Quaternionf rot(imu->orientation.w,
                                        imu->orientation.x,
                                        imu->orientation.y,
                                        imu->orientation.z);
                staged_imu = rot;
            }


        if(pose.topRightCorner(3, 1).isApprox(Eigen::Vector3f::Zero())){
            geometry_msgs::PointStamped::ConstPtr gps = msg.instantiate<geometry_msgs::PointStamped>();;
            if(gps != nullptr){
                Eigen::Vector3f trans(gps->point.x,
                                    gps->point.y,
                                    gps->point.z);
                manager.guessPosition(trans);
            }
        }
        
        //handle point cloud topic
        sensor_msgs::PointCloud2::ConstPtr pc = msg.instantiate<sensor_msgs::PointCloud2>();
        if(pc != nullptr){
            pcl::PointCloud<pcl::PointXYZ>* input_cloud = new PointCloud<pcl::PointXYZ>;
            pcl::fromROSMsg(*pc, *input_cloud);


            manager.feedPC(*input_cloud);
            pose = manager.getPose();

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