# include "ros/ros.h"
# include "geometry_msgs/PoseWithCovarianceStamped.h"

# include <fstream>
# include <Eigen/Dense>
# include <iostream>
using namespace std;

ros::Subscriber sub;

std::string output_file_name = "";
std::string output_file_path = "";
std::string output_file_location = "";

void result_data_callBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){

    std::ofstream output_file;
    // cout << "output file location: " << output_file_location << endl;
    
    Eigen::Quaterniond orientation_matrix(  msg->pose.pose.orientation.w, 
                                            msg->pose.pose.orientation.x, 
                                            msg->pose.pose.orientation.y, 
                                            msg->pose.pose.orientation.z);
    Eigen::Matrix3d eular_matrix(orientation_matrix);

    output_file.open(output_file_location.c_str());
    output_file << msg->header.stamp;
    output_file << ",";
    output_file << msg->pose.pose.position.x;
    output_file << ",";
    output_file << msg->pose.pose.position.y;
    output_file << ",";
    output_file << msg->pose.pose.position.z;
    output_file << ",";
    output_file << eular_matrix(0);
    output_file << ",";
    output_file << eular_matrix(1);
    output_file << ",";
    output_file << eular_matrix(2);
    output_file << "\n";

    return;
}

int main(int argc, char *argv[]){

    ros::init(argc, argv, "csv_output");
    ros::NodeHandle n("~");

    sub = n.subscribe("car_pose", 1, result_data_callBack);

    // input parameters and processing
    // default output file name: "unknown.csv"
    n.param<std::string>("output_filename", output_file_name, "unknown.csv");
    // ROS_INFO("output file name : %s", file_name.c_str());
    // default output file path: "../bag"
    n.param<std::string>("output_path", output_file_path, "../bag");
    // ROS_INFO("output path : %s", output_file_path.c_str());
    output_file_location = output_file_path + "/" + output_file_name;
    // input parameters and processing%

    printf("csv_output initializing done.\n");
    ros::spin();

    return 0;
}