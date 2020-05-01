#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;
using namespace ros;

class read_gdtruth
{
public:
    read_gdtruth(NodeHandle &n);
    void gd_cb(sensor_msgs::PointCloud2);

private:
    Subscriber lidar_sub;
    Publisher map_pub; // lidar_pub;
    ifstream fin;
    rosbag::Bag bag;
    // string line;
    // double read;
    sensor_msgs::PointCloud2 output;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
};

read_gdtruth::read_gdtruth(NodeHandle &n) : cloud(new pcl::PointCloud<pcl::PointXYZ>)
{

    lidar_sub = n.subscribe("/lidar_points", 10, &read_gdtruth::gd_cb, this); ///////////
    map_pub = n.advertise<sensor_msgs::PointCloud2>("lidar_map", 10);         /////////////
    //lidar_pub = n.advertise<sensor_msgs::PointCloud2>("lidar_scan", 10);      ///////////////                             ///////
    fin.open("/bags/ITRI_Public_Ground_truth.csv"); //////////

    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/bags/map.pcd", *cloud) == -1) //* load the file /////////////////
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");                    //////////////////////////
}

void read_gdtruth::gd_cb(sensor_msgs::PointCloud2 msg)
{
    tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    double data[7];

    if (!fin.eof())
    {
        char buffer;

        for (int i = 0; i < 7; i++)
        {
            fin >> data[i] >> buffer;
            cout << setprecision(16) << data[i] << ",";
        }
        cout << endl;
        transform.setOrigin(tf::Vector3(data[1], data[2], data[3]));
        q.setRPY(data[6], data[5], data[4]);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "velodyne"));
        //msg.header.stamp = ros::Time::now();
        //lidar_pub.publish(msg);
        pcl::toROSMsg(*cloud, output);
        output.header.frame_id = "map";
        map_pub.publish(output); /////////////////
    }
}

int main(int argc, char *argv[])
{
    init(argc, argv, "read_gdtruth");
    NodeHandle n;
    read_gdtruth read_gd(n);
    spin();
    return 0;
}