# include <ros/ros.h>
# include <pcl/io/pcd_io.h>
# include <pcl/point_types.h>
# include <pcl/filters/passthrough.h>
# include <pcl_conversions/pcl_conversions.h>
// # include <pcl/point_cloud.h>
// # include <pcl/filters/voxel_grid.h>

# include <sensor_msgs/PointCloud2.h>

# include <iostream>
using namespace std;

ros::Subscriber sub;
ros::Publisher pub_lower;
ros::Publisher pub_mid;
ros::Publisher pub_upper;

void pcl_lower_segmentation(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &input_cloud){

// variable claiming
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
     pcl::PCLPointCloud2 cloud_lower_output;
     sensor_msgs::PointCloud2 msg_lower_output;
// variable claiming%

// filtering a PointCloud using a PassThrough filter
     // Create the filtering object
     pcl::PassThrough<pcl::PointXYZ> pass;
     pass.setInputCloud(input_cloud);
     pass.setFilterFieldName("z");
     // pass.setFilterLimits(__FLT_MIN__, 0.8);
     pass.setFilterLimits(-100, 0.8);
     // pass.setFilterLimitsNegative (true);
     pass.filter(*cloud_filtered);
// filtering a PointCloud using a PassThrough filter%

// processing data type & publish
     pcl::toPCLPointCloud2(*cloud_filtered, cloud_lower_output);
     pcl_conversions::fromPCL(cloud_lower_output, msg_lower_output);
     msg_lower_output.header.frame_id = "/velodyne";
     pub_lower.publish(msg_lower_output);
// processing data type & publish%

}

void pcl_mid_segmentation(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &input_cloud){

// variable claiming
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
     pcl::PCLPointCloud2 cloud_mid_output;
     sensor_msgs::PointCloud2 msg_mid_output;
// variable claiming%

// filtering a PointCloud using a PassThrough filter
     // Create the filtering object
     pcl::PassThrough<pcl::PointXYZ> pass;
     pass.setInputCloud(input_cloud);
     pass.setFilterFieldName("z");
     pass.setFilterLimits(0.8, 1.8);
     //pass.setFilterLimitsNegative (true);
     pass.filter(*cloud_filtered);
// filtering a PointCloud using a PassThrough filter%

// processing data type & publish
     pcl::toPCLPointCloud2(*cloud_filtered, cloud_mid_output);
     pcl_conversions::fromPCL(cloud_mid_output, msg_mid_output);
     msg_mid_output.header.frame_id = "/velodyne";
     pub_mid.publish(msg_mid_output);
// processing data type & publish%

}

void pcl_upper_segmentation(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &input_cloud){
     
// variable claiming
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
     pcl::PCLPointCloud2 cloud_upper_output;
     sensor_msgs::PointCloud2 msg_upper_output;
// variable claiming%

// filtering a PointCloud using a PassThrough filter
     // Create the filtering object
     pcl::PassThrough<pcl::PointXYZ> pass;
     pass.setInputCloud(input_cloud);
     pass.setFilterFieldName("z");
     // pass.setFilterLimits(1.8, __FLT_MAX__);
     pass.setFilterLimits(1.8, 100);
     //pass.setFilterLimitsNegative (true);
     pass.filter(*cloud_filtered);
// filtering a PointCloud using a PassThrough filter%

// processing data type & publish
     pcl::toPCLPointCloud2(*cloud_filtered, cloud_upper_output);
     pcl_conversions::fromPCL(cloud_upper_output, msg_upper_output);
     msg_upper_output.header.frame_id = "/velodyne";
     pub_upper.publish(msg_upper_output);
// processing data type & publish%

}

void horizonSegment_callBack(const sensor_msgs::PointCloud2ConstPtr &input_msg)
{
     pcl::PCLPointCloud2 pcl_pc2;
     pcl_conversions::toPCL(*input_msg, pcl_pc2);
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
     pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

// Fill in the cloud data
     // cloud->width = 5248917;
     // cloud->height = 1;
     // cloud->points.resize(cloud->width * cloud->height);
// Fill in the cloud data%

// call segmentation functions
     pcl_lower_segmentation(cloud);
     pcl_mid_segmentation(cloud);
     pcl_upper_segmentation(cloud);
// call segmentation functions%

}

int main(int argc, char **argv)
{
     ros::init(argc, argv, "pcl_horizonal_segmentation");
     ros::NodeHandle n;

     sub = n.subscribe("/lidar_points", 1, horizonSegment_callBack);
     pub_lower = n.advertise<sensor_msgs::PointCloud2>("/lower_points", 1);
     pub_mid = n.advertise<sensor_msgs::PointCloud2>("/mid_points", 1);
     pub_upper = n.advertise<sensor_msgs::PointCloud2>("/upper_points", 1);

// for debugging

// read point cloud data from PCD files
     // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

     // //* load the file
     // if (pcl::io::loadPCDFile<pcl::PointXYZ>("/bags/itri/map.pcd", *cloud) == -1) 
     // {
     //      PCL_ERROR("Couldn't read file map.pcd \n");
     //      return (-1);
     // }
     // std::cout << "Loaded "
     //           << "width: " << cloud->width << ", height" << cloud->height << "\n"
     //           << cloud->width * cloud->height
     //           << " data points from map.pcd with the following fields: "
     //           << std::endl;
     // // for (std::size_t i = 0; i < cloud->points.size(); ++i)
     // //      std::cout << "    " << cloud->points[i].x
     // //                << " " << cloud->points[i].y
     // //                << " " << cloud->points[i].z << std::endl;
// read point cloud data from PCD files%

// filtering a PointCloud using a PassThrough filter
     // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
     // pcl::PCLPointCloud2 cloud_lower_output;
     // pcl::PCLPointCloud2 cloud_mid_output;
     // pcl::PCLPointCloud2 cloud_upper_output;
     // sensor_msgs::PointCloud2 msg_lower_output;
     // sensor_msgs::PointCloud2 msg_mid_output;
     // sensor_msgs::PointCloud2 msg_upper_output;

     // // Create the filtering object
     // pcl::PassThrough<pcl::PointXYZ> pass;
     // pass.setInputCloud(cloud);
     // pass.setFilterFieldName("z");
     // pass.setFilterLimits(0.0, 1.0);
     // //pass.setFilterLimitsNegative (true);
     // pass.filter(*cloud_filtered);

     // pcl::toPCLPointCloud2(*cloud_filtered, cloud_lower_output);
     // pcl_conversions::fromPCL(cloud_lower_output, msg_lower_output);
     // pub_lower.publish(msg_lower_output);

     // // std::cerr << "Cloud after filtering: " << std::endl;
     // // for (std::size_t i = 0; i < cloud_filtered->points.size(); ++i)
     // //      std::cerr << "    " << cloud_filtered->points[i].x << " "
     // //                << cloud_filtered->points[i].y << " "
     // //                << cloud_filtered->points[i].z << std::endl;
// filtering a PointCloud using a PassThrough filter%

// for debugging%

     cout << "pcl_horizonal_segmentation initializing done." << endl;
     ros::spin();

     return 0;
}
