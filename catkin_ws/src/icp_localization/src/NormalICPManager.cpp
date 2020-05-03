#include"NormalICPManager.hpp"
using namespace pcl;
NormalICPManager::NormalICPManager():pose(Eigen::Matrix4d::Identity()), 
                                        guess(Eigen::Matrix4d::Identity()), 
                                        map_cloud(new pcl::PointCloud<pcl::PointXYZ>), 
                                        map_with_normals(new PointCloud<PointXYZRGBNormal>){
}

NormalICPManager::NormalICPManager(char const * map_file):pose(Eigen::Matrix4d::Identity()), 
                                                            guess(Eigen::Matrix4d::Identity()), 
                                                            map_cloud(new pcl::PointCloud<pcl::PointXYZ>), 
                                                            map_with_normals(new PointCloud<PointXYZRGBNormal>){
    if(pcl::io::loadPCDFile (map_file, *map_cloud) < 0){
        PCL_ERROR("Couldn't read file %s\n", map_file);
    }
    computeNormal(map_cloud, map_with_normals);
}


void NormalICPManager::loadMap(std::string map_file){
    if(pcl::io::loadPCDFile (map_file, *map_cloud) < 0){
        PCL_ERROR("Couldn't read file %s\n", map_file);
    }
    computeNormal(map_cloud, map_with_normals);
    return;
}

void NormalICPManager::feedPC(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud){
    pcl::PointCloud<PointXYZRGBNormal>::Ptr input_with_normals(new PointCloud<PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal> final_cloud;
    Eigen::Matrix4d gs;

    computeNormal(input_cloud, input_with_normals);

    icp.setInputSource(input_with_normals);
    

    if(! guess.isApprox(Eigen::Matrix4d::Identity())){
        gs = guess;
        guess.setIdentity();
    }
    else if(! pose.isApprox(Eigen::Matrix4d::Identity())){
        gs = pose;
    }else{
        PCL_WARN("Alining without guess\n");
        gs.setIdentity();
    }

    if(gs.isApprox(Eigen::Matrix4d::Identity()))
        icp.setInputTarget(map_with_normals);
    else{
        Eigen::Vector3d point = gs.topRightCorner(3, 1);
        pcl::PointCloud<PointXYZRGBNormal>::Ptr inputTarget(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        this->selectMapRange(point(0), point(1), point(2), 50, 50, 30, inputTarget);
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

void NormalICPManager::selectMapRange(float x_center, float y_center, float z_center, float x_length, float y_length, float z_length, pcl::PointCloud<PointXYZRGBNormal>::Ptr& result){
    pcl::PassThrough<PointXYZRGBNormal> pass;
    static PointCloud<PointXYZRGBNormal>::Ptr _x_filted(new PointCloud<PointXYZRGBNormal>), 
                                                _y_filted(new PointCloud<PointXYZRGBNormal>);

    // std::cout << "filtering: " << "x=" << x_center << ", y=" << y_center << ", z=" << z_center << std::endl;
    // filtering X axis
    pass.setInputCloud(map_with_normals);
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
    // std::cout << "passthough done" << std::endl;
    std::cout << result->width << std::endl; 
    return;

}

void NormalICPManager::computeNormal(PointCloud<PointXYZ>::Ptr input, PointCloud<PointXYZRGBNormal>::Ptr output){
    
    pcl::PointCloud<pcl::Normal>::Ptr normals ( new pcl::PointCloud<pcl::Normal> );
    pcl::search::KdTree<pcl::PointXYZ>::Ptr searchTree (new pcl::search::KdTree<pcl::PointXYZ>);
    searchTree->setInputCloud ( input );
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimator;
    normalEstimator.setInputCloud ( input );
    normalEstimator.setSearchMethod ( searchTree );
    normalEstimator.setKSearch ( 15 );
    normalEstimator.compute ( *normals );
  
    pcl::concatenateFields( *input, *normals, *output );

}