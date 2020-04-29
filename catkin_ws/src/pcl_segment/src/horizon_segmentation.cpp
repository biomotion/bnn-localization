# include <ros/ros.h>
# include <pcl/io/pcd_io.h>
# include <pcl/point_types.h>
# include <pcl/filters/passthrough.h>
// # include <pcl/point_cloud.h>
// # include <pcl_conversions/pcl_conversions.h>
// # include <pcl/filters/voxel_grid.h>

# include <sensor_msgs/PointCloud2.h>

# include <iostream>
using namespace std;

ros::Subscriber sub;
ros::Publisher pub_low;
ros::Publisher pub_mid;
ros::Publisher pub_high;

void horizonSegment_callBack(const sensor_msgs::PointCloud2ConstPtr &input_msg)
{
// read point cloud data from PCD files
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

     //* load the file
     if (pcl::io::loadPCDFile<pcl::PointXYZ>("../../bags/map.pcd", *cloud) == -1) 
     {
          PCL_ERROR("Couldn't read file test_pcd.pcd \n");
          return;
     }
     std::cout << "Loaded "
               << cloud->width * cloud->height
               << " data points from test_pcd.pcd with the following fields: "
               << std::endl;
     // for (std::size_t i = 0; i < cloud->points.size(); ++i)
     //      std::cout << "    " << cloud->points[i].x
     //                << " " << cloud->points[i].y
     //                << " " << cloud->points[i].z << std::endl;
// read point cloud data from PCD files%


// filtering a PointCloud using a PassThrough filter
     // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

     // Fill in the cloud data
     cloud->width = 10;
     cloud->height = 10;
     cloud->points.resize(cloud->width * cloud->height);

     for (std::size_t i = 0; i < cloud->points.size(); ++i)
     {
          cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
          cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
          cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
     }

     std::cerr << "Cloud before filtering: " << std::endl;
     for (std::size_t i = 0; i < cloud->points.size(); ++i)
          std::cerr << "    " << cloud->points[i].x << " "
                    << cloud->points[i].y << " "
                    << cloud->points[i].z << std::endl;

     // Create the filtering object
     pcl::PassThrough<pcl::PointXYZ> pass;
     pass.setInputCloud(cloud);
     pass.setFilterFieldName("z");
     pass.setFilterLimits(0.0, 1.0);
     //pass.setFilterLimitsNegative (true);
     pass.filter(*cloud_filtered);

     std::cerr << "Cloud after filtering: " << std::endl;
     for (std::size_t i = 0; i < cloud_filtered->points.size(); ++i)
          std::cerr << "    " << cloud_filtered->points[i].x << " "
                    << cloud_filtered->points[i].y << " "
                    << cloud_filtered->points[i].z << std::endl;
// filtering a PointCloud using a PassThrough filter%
}

int main(int argc, char **argv)
{
     ros::init(argc, argv, "pcl_horizonal_segmentation");
     ros::NodeHandle n;

     sub = n.subscribe("/map", 1, horizonSegment_callBack);
     pub_low = n.advertise<sensor_msgs::PointCloud2>("/lower_points", 1);
     pub_mid = n.advertise<sensor_msgs::PointCloud2>("/mid_points", 1);
     pub_high = n.advertise<sensor_msgs::PointCloud2>("/upper_points", 1);

     cout << "pcl_horizonal_segmentation initializing done." << endl;
     ros::spin();

     return 0;
}
