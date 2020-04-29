#include"ICPManager.hpp"
using namespace pcl;
ICPManager::ICPManager():pose(Eigen::Matrix4f::Identity()), guess(Eigen::Matrix4f::Identity()), map_cloud(new pcl::PointCloud<pcl::PointXYZ>){
    // this->icp.setMaxCorrespondenceDistance(0.05);
    // this->icp.setTransformationEpsilon(1e-10);
    // this->icp.setEuclideanFitnessEpsilon(0.01);
    // this->icp.setMaximumIterations(100);   
}

ICPManager::ICPManager(char const * map_file):pose(Eigen::Matrix4f::Identity()), guess(Eigen::Matrix4f::Identity()), map_cloud(new pcl::PointCloud<pcl::PointXYZ>){
    if(pcl::io::loadPCDFile (map_file, *map_cloud) < 0){
        PCL_ERROR("Couldn't read file %s\n", map_file);
    }
    this->icp.setMaxCorrespondenceDistance(100);
    this->icp.setTransformationEpsilon(1e-10);
    this->icp.setEuclideanFitnessEpsilon(0.001);
    this->icp.setMaximumIterations(100);   
}


void ICPManager::loadMap(std::string map_file){
    if(pcl::io::loadPCDFile (map_file, *map_cloud) < 0){
        PCL_ERROR("Couldn't read file %s\n", map_file);
    }
    return;
}

void ICPManager::feedPC(pcl::PointCloud<pcl::PointXYZ>& input_cloud){
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_ptr(&input_cloud);
    pcl::PointCloud<pcl::PointXYZ> final_cloud;
    Eigen::Matrix4f gs;

    icp.setInputSource(input_ptr);
    

    if(! guess.isApprox(Eigen::Matrix4f::Identity())){
        gs = guess;
        guess.setIdentity();
    }
    else if(! pose.isApprox(Eigen::Matrix4f::Identity())){
        gs = pose;
    }else{
        PCL_WARN("Alining without guess\n");
        gs.setIdentity();
    }

    if(gs.isApprox(Eigen::Matrix4f::Identity()))
        icp.setInputTarget(map_cloud);
    else{
        Eigen::Vector3f point = gs.topRightCorner(3, 1);
        pcl::PointCloud<PointXYZ>::Ptr inputTarget(new pcl::PointCloud<pcl::PointXYZ>);
        this->selectMapRange(point(0), point(1), point(2), 50, 50, 50, inputTarget);
        icp.setInputTarget(inputTarget);
    }
    std::cout << "aligning..." << std::endl;
    icp.align(final_cloud, gs);

    std::cout << "has converge: " << icp.hasConverged() << std::endl; 
    if(icp.hasConverged()){
        std::cout << "score: " << icp.getFitnessScore() << std::endl; 
        std::cout << icp.getFinalTransformation() << std::endl;
        pose = icp.getFinalTransformation();
        PCL_INFO("Pose updated\n");
    }
}

void ICPManager::selectMapRange(float x_center, float y_center, float z_center, float x_length, float y_length, float z_length, pcl::PointCloud<PointXYZ>::Ptr& result){
    pcl::PassThrough<pcl::PointXYZ> pass;
    static pcl::PointCloud<pcl::PointXYZ>::Ptr _x_filted(new pcl::PointCloud<pcl::PointXYZ>), 
                                                _y_filted(new pcl::PointCloud<pcl::PointXYZ>);

    std::cout << "filtering: " << "x=" << x_center << ", y=" << y_center << ", z=" << z_center << std::endl;
    // filtering X axis
    pass.setInputCloud(pcl::PointCloud<PointXYZ>::Ptr(map_cloud));
    pass.setFilterFieldName("x");
    pass.setFilterLimits(x_center - x_length/2, x_center + x_length/2);
    pass.filter(*_x_filted);
    // filtering Y axis
    pass.setInputCloud(_x_filted);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(y_center - y_length/2, y_center + y_length/2);
    pass.filter(*_y_filted);
    // filtering Z axis
    pass.setInputCloud(_y_filted);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z_center - z_length/2, z_center + z_length/2);
    pass.filter(*result);
    std::cout << "passthough done" << std::endl;
    std::cout << result->width << std::endl; 
    return;

}
