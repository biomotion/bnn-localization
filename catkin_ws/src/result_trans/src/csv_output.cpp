# include "ros/ros.h"
# include "geometry_msgs/PoseStamped.h"
#include <tf_conversions/tf_eigen.h>

# include <fstream>
# include <Eigen/Dense>
# include <iostream>
# include <cstdio>
#include <stdio.h>
# include <string>
using namespace std;

ros::Subscriber sub;
std::ofstream output_file;
std::string output_file_name = "";
std::string output_file_path = "";
std::string output_file_location = "";

void result_data_callBack(const geometry_msgs::PoseStamped::ConstPtr& msg){
    std::cout << "got message" << std::endl;
    // std::ofstream output_file;
    // cout << "output file location: " << output_file_location << endl;
    
    Eigen::Quaterniond orientation_matrix(  msg->pose.orientation.w, 
                                            msg->pose.orientation.x, 
                                            msg->pose.orientation.y, 
                                            msg->pose.orientation.z);
    Eigen::Matrix3d eular_matrix(orientation_matrix);

    output_file << setprecision(16);
    output_file << msg->header.stamp
                << ","
                << msg->pose.position.x
                << ","
                << msg->pose.position.y
                << ","
                << msg->pose.position.z
                << ","
                << eular_matrix.eulerAngles(0,1,2)(2)
                << ","
                << eular_matrix.eulerAngles(0,1,2)(1)
                << ","
                << eular_matrix.eulerAngles(0,1,2)(0)
                << "\n";
    output_file.flush();

    return;
}

int main(int argc, char *argv[]){

    ros::init(argc, argv, "csv_output");
    ros::NodeHandle n("~");

    sub = n.subscribe("/car_pose", 1, result_data_callBack);

    // input parameters and processing
    // default output file name: "unknown.csv"
    n.param<std::string>("output_file", output_file_name, "/bags/unknown.csv");

    output_file_location = output_file_name;
    // input parameters and processing%
    output_file.open(output_file_location.c_str(), std::ios_base::out);
    printf("csv_output initializing done.\n");

    while(ros::ok())
        ros::spinOnce();
    output_file.close();
    

    return 0;
}