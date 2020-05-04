#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <ros/ros.h>

using namespace std;
using namespace ros;

class Icp_ekf
{
public:
    Icp_ekf();
    Icp_ekf(NodeHandle &);
    void ekf_cb(geometry_msgs::PoseStamped);
    void com_cb(const geometry_msgs::PoseWithCovarianceStamped&);

private:
    Subscriber pose_sub, imu_sub,com_sub;
    Publisher vo_pub,com_pub;
    float cov_x, cov_y, cov_z;
    float cov_yaw, cov_pitch, cov_roll;
    int id;
};

Icp_ekf::Icp_ekf() {}

Icp_ekf::Icp_ekf(NodeHandle &n)
{
    pose_sub = n.subscribe("/car_pose", 10, &Icp_ekf::ekf_cb, this);
    //imu_sub = n.subsciber("/imu/data",10,&Icp_ekf::imu)
    vo_pub = n.advertise<nav_msgs::Odometry>("/vo", 10);
    com_sub = n.subscribe("/robot_pose_ekf/odom_combined",10,&Icp_ekf::com_cb,this);
    com_pub = n.advertise<geometry_msgs::PoseStamped>("/car_combine", 10);
    n.param<float>("cov_x", cov_x, 0.0);
    n.param<float>("cov_y", cov_y, 0.0);
    n.param<float>("cov_z", cov_z, 0.0);
    n.param<float>("cov_yaw", cov_yaw, 0.0);
    n.param<float>("cov_pitch", cov_pitch, 0.0);
    n.param<float>("cov_roll", cov_roll, 0.0);
    id = 0;
}

void Icp_ekf::ekf_cb(geometry_msgs::PoseStamped msg)
{
    nav_msgs::Odometry odem;
    odem.header.stamp = msg.header.stamp;
    odem.header.frame_id = "map";
    odem.header.seq = id++;
    odem.pose.pose.position.x = msg.pose.position.x;
    odem.pose.pose.position.y = msg.pose.position.y;
    odem.pose.pose.position.z = msg.pose.position.z;
    odem.pose.pose.orientation.x = msg.pose.orientation.x;
    odem.pose.pose.orientation.y = msg.pose.orientation.y;
    odem.pose.pose.orientation.z = msg.pose.orientation.z;
    odem.pose.pose.orientation.w = msg.pose.orientation.w;

    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            if (i != j)
                odem.pose.covariance[i * 6 + j] = 0;
        }
    }
    odem.pose.covariance[0] = cov_x;
    odem.pose.covariance[7] = cov_y;
    odem.pose.covariance[14] = cov_z;
    odem.pose.covariance[21] = cov_yaw;
    odem.pose.covariance[28] = cov_pitch;
    odem.pose.covariance[35] = cov_roll;
    vo_pub.publish(odem);
    cout << "setup" << endl;
}

void Icp_ekf::com_cb(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = ros::Time(0);
    pose.pose = msg.pose.pose;
    com_pub.publish(pose);
}

int main(int argc, char *argv[])
{
    init(argc, argv, "Icp_ekf");
    NodeHandle n("~");
    Icp_ekf icp_ekf(n);
    spin();
    return 0;
}