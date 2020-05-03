#include<ros/ros.h>

#include<sensor_msgs/PointCloud2.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>

#include<pcl/filters/passthrough.h>
using namespace pcl;
using namespace std;



ros::Publisher pub;

float z_min, z_max;

void cb_cloud(const sensor_msgs::PointCloud2::ConstPtr& input_msg){
    ROS_INFO("CB");
	sensor_msgs::PointCloud2 output_msg;
	pcl::PointCloud<PointXYZ>::Ptr input_cloud(new pcl::PointCloud<PointXYZ>);
	pcl::PointCloud<PointXYZ>::Ptr output_cloud(new pcl::PointCloud<PointXYZ>);

    pcl::fromROSMsg(*input_msg, *input_cloud);

	std::cout << "before filtered" << std::endl;
	std::cout << *input_cloud << std::endl;

	pcl::PassThrough<PointXYZ> pass;
	pass.setInputCloud(input_cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(z_min, z_max);
	pass.filter(*output_cloud);

	std::cout << "after filtered" << std::endl;
	std::cout << *output_cloud << std::endl;

	pcl::toROSMsg(*output_cloud, output_msg);
	output_msg.header.frame_id = input_msg->header.frame_id;
	pub.publish(output_msg);

}

int main(int argc, char** argv){
	ros::init(argc, argv, "ground_filter_node");
	ros::NodeHandle nh("~");

	nh.param<float>("z_min", z_min, -0.2f);
	nh.param<float>("z_max", z_max, 0.2f);

	cout << z_min << endl
		 << z_max << endl;
	ros::Subscriber sub = nh.subscribe("points_in", 1, cb_cloud);
	pub = nh.advertise<sensor_msgs::PointCloud2> ("points_out", 1);

	ros::spin();
	return 0;
}
