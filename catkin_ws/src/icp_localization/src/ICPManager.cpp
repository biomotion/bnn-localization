#include"ICPManager.hpp"

// ICPManager::ICPManager():pose(Eigen::Matrix4f::Identity()), guess(Eigen::Matrix4f::Identity()){
//     this->icp.setMaxCorrespondenceDistance(0.05);
//     this->icp.setTransformationEpsilon(1e-10);
//     this->icp.setEuclideanFitnessEpsilon(0.01);
//     this->icp.setMaximumIterations(100);   
// }

ICPManager::ICPManager(char const * map_file):pose(Eigen::Matrix4f::Identity()), guess(Eigen::Matrix4f::Identity()){
    this->map_cloud = new pcl::PointCloud<pcl::PointXYZ>;
    if(pcl::io::loadPCDFile (map_file, *map_cloud) < 0){
        PCL_ERROR("Couldn't read file %s\n", map_file);
    }
    this->icp.setMaxCorrespondenceDistance(0.05);
    this->icp.setTransformationEpsilon(1e-5);
    this->icp.setEuclideanFitnessEpsilon(0.01);
    this->icp.setMaximumIterations(10);   
}


void ICPManager::loadMap(std::string map_file){
    this->map_cloud = new pcl::PointCloud<pcl::PointXYZ>;
    if(pcl::io::loadPCDFile (map_file, *map_cloud) < 0){
        PCL_ERROR("Couldn't read file %s\n", map_file);
    }
    return;
}

void ICPManager::feedPC(pcl::PointCloud<pcl::PointXYZ>& input_cloud){
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_ptr(&input_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_ptr(map_cloud);
    pcl::PointCloud<pcl::PointXYZ> final_cloud;

    icp.setInputSource(input_ptr);
    icp.setInputTarget(map_ptr);

    if(! guess.isApprox(Eigen::Matrix4f::Identity())){
        PCL_INFO("Calculating... \n");
        icp.align(final_cloud, guess);
        guess.setIdentity();
    }
    else if(! pose.isApprox(Eigen::Matrix4f::Identity())){
        icp.align(final_cloud, pose);
    }else{
        PCL_WARN("Dropping this frame\n");
        return;
    }
    std::cout << "has converge: " << icp.hasConverged() << std::endl; 
    if(icp.hasConverged()){
        std::cout << "score: " << icp.getFitnessScore() << std::endl; 
        std::cout << icp.getFinalTransformation() << std::endl;
        pose = icp.getFinalTransformation();
        PCL_INFO("Pose updated\n");
    }
}

