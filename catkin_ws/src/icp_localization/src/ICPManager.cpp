#include"ICPManager.hpp"
using namespace pcl;
ICPManager::ICPManager():pose(Eigen::Matrix4d::Identity()), guess(Eigen::Matrix4d::Identity()), map_cloud(new pcl::PointCloud<pcl::PointXYZ>){
}

ICPManager::ICPManager(char const * map_file):pose(Eigen::Matrix4d::Identity()), guess(Eigen::Matrix4d::Identity()), map_cloud(new pcl::PointCloud<pcl::PointXYZ>){
    if(pcl::io::loadPCDFile (map_file, *map_cloud) < 0){
        PCL_ERROR("Couldn't read file %s\n", map_file);
    }
}


void ICPManager::loadMap(std::string map_file){
    if(pcl::io::loadPCDFile (map_file, *map_cloud) < 0){
        PCL_ERROR("Couldn't read file %s\n", map_file);
    }
    return;
}

void ICPManager::feedPC(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud){
    pcl::PointCloud<pcl::PointXYZ> final_cloud;
    Eigen::Matrix4d gs;

    pcl::PointCloud<PointXYZ>::Ptr input_processed(new PointCloud<PointXYZ>);
    this->pointsPreCompute(input_cloud, input_processed);
    
    // align with guess
    if(! guess.isApprox(Eigen::Matrix4d::Identity())){ 
        gs = guess;
        guess.setIdentity();
        Eigen::Vector3d point = gs.topRightCorner(3, 1);

        pcl::PointCloud<PointXYZ>::Ptr inputTarget(new pcl::PointCloud<pcl::PointXYZ>);
        this->selectMapRange(point(0), point(1), point(2), 50, 50, 30, inputTarget);
        icp.setInputSource(input_processed);
        icp.setInputTarget(inputTarget);
    }
    // align with pose as guess
    else if(! pose.isApprox(Eigen::Matrix4d::Identity())){
        gs = pose;
        Eigen::Vector3d point = gs.topRightCorner(3, 1);
        pcl::PointCloud<PointXYZ>::Ptr inputTarget(new pcl::PointCloud<pcl::PointXYZ>);
        this->selectMapRange(point(0), point(1), point(2), 50, 50, 30, inputTarget);
        icp.setInputSource(input_processed);
        icp.setInputTarget(inputTarget);
    }
    // align without any guess using whole map
    else{
        PCL_WARN("Alining without guess\n");
        gs.setIdentity();
        icp.setInputSource(input_processed);
        icp.setInputTarget(map_cloud);
    }

    std::cout << "aligning..." << std::endl;
    icp.align(final_cloud, gs); 

    std::cout << "has converge: " << icp.hasConverged() << std::endl; 
    if(icp.hasConverged()){
        std::cout << "score: " << icp.getFitnessScore(1.0f) << std::endl; 
        std::cout << icp.getFinalTransformation() << std::endl;
        pose = icp.getFinalTransformation();
        PCL_INFO("Pose updated\n");
    }
}

void ICPManager::selectMapRange(float x_center, float y_center, float z_center, float x_length, float y_length, float z_length, pcl::PointCloud<PointXYZ>::Ptr& result){
    pcl::PassThrough<PointXYZ> pass;
    pcl::VoxelGrid<PointXYZ> down;
    StatisticalOutlierRemoval<PointXYZ> noise_filter;

    // std::cout << "filtering: " << "x=" << x_center << ", y=" << y_center << ", z=" << z_center << std::endl;
    // filtering X axis
    pass.setInputCloud(map_cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(x_center - x_length/2, x_center + x_length/2);
    pass.filter(*result);
    // filtering Y axis
    pass.setInputCloud(result);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(y_center - y_length/2, y_center + y_length/2);
    pass.filter(*result);
    // filtering Z axis
    pass.setInputCloud(result);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z_center - z_length/2, z_center + z_length/2);
    pass.filter(*result);
    // std::cout << "passthough done" << std::endl;
    std::cout << result->width << std::endl; 
    noise_filter.setInputCloud(result);
    noise_filter.setMeanK(64);
    noise_filter.setStddevMulThresh(1.0);
    noise_filter.filter(*result);
    // down.setInputCloud(result);
    // down.setLeafSize(0.1, 0.1, 0.1);
    // down.filter(*result);
    // std::cout << result->width << std::endl;
    return;

}

void ICPManager::pointsPreCompute(PointCloud<PointXYZ>::Ptr input, PointCloud<PointXYZ>::Ptr output){
    pcl::VoxelGrid<pcl::PointXYZ> grid_filter;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> noise_filter;
    pcl::PassThrough<PointXYZ> pass;
    noise_filter.setInputCloud(input);
    noise_filter.setMeanK(64);
    noise_filter.setStddevMulThresh(1.0);
    noise_filter.filter(*output);
    // grid_filter.setInputCloud(output);
    // grid_filter.setLeafSize(0.1f, 0.1f, 0.1f);
    // grid_filter.filter(*output);
    // pass.setInputCloud(output);
    // pass.setFilterFieldName("y");
    // pass.setFilterLimits(-3, 0);
    // pass.setNegative(true);
    // pass.filter(*output);

}

void ICPManager::groundFilter(float zmin, float zmax, PointCloud<PointXYZ>::Ptr input, PointCloud<PointXYZ>::Ptr output){
    pcl::PassThrough<PointXYZ> pass;

    // filtering Z axis
    pass.setInputCloud(input);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(zmin, zmax);
    pass.filter(*output);

    std::cout << "ground points: " << output->width << std::endl;
    return;
}