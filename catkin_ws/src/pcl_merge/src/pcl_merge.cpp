# include <pcl/io/pcd_io.h>
# include <pcl/point_types.h>
# include <pcl/filters/passthrough.h>
# include <pcl_conversions/pcl_conversions.h>
# include <pcl/io/pcd_io.h>
# include <sensor_msgs/PointCloud2.h>

# include <dirent.h>
# include <sys/types.h>
# include <vector>
# include <iostream>
using namespace std;

vector<string> filename_vec;
std::string output_filename = "nuscenes.pcd";

void read_filename(const char *path) {
    struct dirent *entry;
    DIR *dir = opendir(path);
   
    if (dir == NULL) {
        return;
    }
    while ((entry = readdir(dir)) != NULL) {
        // std::cout << entry->d_name << std::endl;
        filename_vec.push_back(entry->d_name);
    }
    closedir(dir);

    std::cout << "Read: " << (filename_vec.size() - 2) << " files\n";
    // for(int i = 2; i < filename_vec.size(); i++){
    //     std::cout << filename_vec[i] << std::endl;
    // }
}

void merge_pcl(){

    char *file_dir = "/bags/nuscenes/map";
    read_filename(file_dir);
// read point cloud data from PCD files
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    std::string file_location;
    // //* load the file
    for(int count = 2; count < filename_vec.size(); count++){
        file_location = std::string(file_dir) + "/" + filename_vec[count];
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_location, *temp_cloud) == -1) 
        {
            PCL_ERROR("Couldn't read file.");
            // std::cout << "filename: " << file_location << std::endl;
            return;
        }
        *cloud += *temp_cloud;
        std::cout << "Loaded "
                << "width: " << temp_cloud->width << ", height: " << temp_cloud->height << "\n"
                << temp_cloud->width * temp_cloud->height
                << " data points from " << filename_vec[count] << " with the following fields: "
                << std::endl;
    }
    // //* load the file%
    std::cout << "--------------------------------------------------------------\n";
    std::cout << "Loaded "
            << "width: " << cloud->width << ", height" << cloud->height << "\n"
            << cloud->width * cloud->height
            << " data points from map.pcd with the following fields: "
            << std::endl;    
    
    // for (std::size_t i = 0; i < cloud->points.size(); ++i)
    //      std::cout << "    " << cloud->points[i].x
    //                << " " << cloud->points[i].y
    //                << " " << cloud->points[i].z << std::endl;
// read point cloud data from PCD files%

// write to pcd file
    pcl::io::savePCDFileASCII (output_filename, *cloud);
    std::cout << "Saved " << cloud->points.size () << " data points to " << output_filename  << "." << std::endl;
// write to pcd file%
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pcl_horizonal_segmentation");
    ros::NodeHandle n;

    merge_pcl();

   return 0;
}