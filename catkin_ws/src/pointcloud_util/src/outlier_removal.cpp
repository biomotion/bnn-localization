#include<ros/ros.h>

#include<sensor_msgs/PointCloud2.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
using namespace pcl;

ros::Publisher pub;

void cb_cloud(const sensor_msgs::PointCloud2ConstPtr& input_msg){
    ROS_INFO("CB");
	sensor_msgs::PointCloud2 output_msg;
	pcl::PointCloud<PointXYZ>::Ptr input_cloud(new pcl::PointCloud<PointXYZ>);
	pcl::PointCloud<PointXYZ>::Ptr output_cloud(new pcl::PointCloud<PointXYZ>);

    pcl::fromROSMsg(*input_msg, *input_cloud);


    pcl::StatisticalOutlierRemoval<PointXYZ> filter;
    filter.setInputCloud(input_cloud);
    filter.setMeanK(128);
    filter.setStddevMulThresh(0.1);
    filter.filter(*output_cloud);

	pcl::toROSMsg(*output_cloud, output_msg);
	output_msg.header.frame_id = input_msg->header.frame_id;
	pub.publish(output_msg);

}

int main(int argc, char** argv){
	ros::init(argc, argv, "outlier_removal_node");
	ros::NodeHandle nh("~");


	ros::Subscriber sub = nh.subscribe("points_in", 10, cb_cloud);
	pub = nh.advertise<sensor_msgs::PointCloud2> ("points_out", 1);

	ros::spin();
	return 0;
}
