#include<ros/ros.h>

#include<sensor_msgs/PointCloud2.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>


ros::Publisher pub;

void cb_cloud(const sensor_msgs::PointCloud2ConstPtr& input_msg){
	sensor_msgs::PointCloud2 output;
	pcl::PCLPointCloud2* input_cloud = new pcl::PCLPointCloud2; 
	pcl::PCLPointCloud2ConstPtr cloudPtr(input_cloud);
	pcl::PCLPointCloud2 cloud_filtered;

	pcl_conversions::toPCL(*input_msg, *input_cloud);


	//process here
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud (cloudPtr);
	sor.setLeafSize (1.6f, 1.6f, 1.6f);
	sor.filter (cloud_filtered);

	pcl_conversions::fromPCL(cloud_filtered, output);
	
	pub.publish(output);

}

int main(int argc, char** argv){
	ros::init(argc, argv, "pcl_downsample_node");
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("/map", 1, cb_cloud);
	pub = nh.advertise<sensor_msgs::PointCloud2> ("/map_out", 1);

	ros::spin();
	return 0;
}
