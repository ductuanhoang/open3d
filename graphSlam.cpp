
#include "graphSlam.h"
#include "open3d/Open3D.h"
#include "open3d/pipelines/registration/GlobalOptimization.h"
#include "open3d/pipelines/registration/PoseGraph.h"
// #include "keyFrame.h"

/**
 * @brief
 *
 * @param cloud1
 * @param cloud2
 * @param voxel_size
 * @return open3d::pipelines::registration::RegistrationResult
 */
extern open3d::pipelines::registration::RegistrationResult match(open3d::geometry::PointCloud &cloud1,
                                                                 open3d::geometry::PointCloud &cloud2,
                                                                 double voxel_size);

graphSlam::graphSlam(/* args */)
{
    std::cout << "graphSlam initialized" << std::endl;
    // this->map_cloud = open3d::geometry::PointCloud();
    map_cloud = open3d::geometry::PointCloud();
    graph = (open3d::pipelines::registration::PoseGraph());
}

graphSlam::~graphSlam()
{
    std::cout << "graphSlam deleted" << std::endl;
}

/**
 * @brief
 *
 * @param cloud
 * @param voxel_size
 * @param export_map
 * @param visualize
 */


void graphSlam::update(open3d::geometry::PointCloud &cloud,
                        double voxel_size,
                        bool export_map,
                        bool visualize)
{
    keyFrame kf;
    if (keyframes.size() == 0)
    {
        std::cout << "keyframes.size() = "<< keyframes.size() << std::endl;
        // std::vector<open3d::geometry::PointCloud>  kf = new keyFrame(0, cloud, );
        Eigen::Matrix4d M = Eigen::Matrix4d::Identity();
        
        kf = keyFrame(0, cloud, M);
        keyframes.push_back(kf);
        graph.nodes_.push_back(keyframes.at(keyframes.size() - 1).node);
    }
    else
    {
        open3d::pipelines::registration::RegistrationResult result_icp = match(cloud, keyframes.at(keyframes.size() - 1).cloud, voxel_size);
        Eigen::Matrix4d odom = (Eigen::Matrix4d )result_icp.transformation_.dot( keyframes.at(keyframes.size() - 1).odom );
        // Eigen::Vector4d point = Eigen::Vector4d(1.0, 2.0, 3.0, 1.0);
        // const Eigen::Vector4d plane_model;
        // double odom = result_icp.transformation_.dot(point);

        // auto odom = result_icp.transformation_.dot();
        // std::cout << "odom = " << odom << std::endl;

        Eigen::Matrix4d odom2 = Eigen::Matrix4d::Identity();

        kf = keyFrame(keyframes.size(), cloud, odom2);
        graph.nodes_.push_back(keyframes[keyframes.size() - 1].node);
        keyframes.push_back(kf);
        Eigen::Matrix6d information = open3d::pipelines::registration::GetInformationMatrixFromPointClouds(keyframes.at(keyframes.size() - 2).cloud,
                                                                                                            keyframes.at(keyframes.size() - 1).cloud,
                                                                                                            voxel_size * 1.4,
                                                                                                            result_icp.transformation_);

        open3d::pipelines::registration::PoseGraphEdge edge = open3d::pipelines::registration::PoseGraphEdge(keyframes.at(keyframes.size() - 1).id,
                                                                                                            keyframes.at(keyframes.size() - 2).id,
                                                                                                            result_icp.transformation_,
                                                                                                            information,
                                                                                                            false);
        graph.edges_.push_back(edge);
        std::cout << "keyframes size: " << keyframes.size() << std::endl;
    }
    // TODO:
    open3d::geometry::PointCloud transformed {kf.cloud}; // deep copy 
    
    kf.cloud.Transform(kf.node.pose_);
    srand (time(NULL));
    Eigen::Vector3d color((double) rand()/RAND_MAX, (double) rand()/RAND_MAX, (double) rand()/RAND_MAX);
    kf.cloud.PaintUniformColor(color);

    if( visualize == true)
    {   
        std::cout << "map_cloud size 1 =  " << this->map_cloud.points_.size()<<std::endl;
        std::cout << "transformed = " << kf.cloud.VoxelDownSample(voxel_size/4)->points_.size()<< " points" << std::endl;
        this->map_cloud += *kf.cloud.VoxelDownSample(voxel_size/4);
        std::cout << "map_cloud size 2 =  " << this->map_cloud.points_.size()<<std::endl;

        // std::shared_ptr<open3d::geometry::PointCloud> pointcloud_ptr(new open3d::geometry::PointCloud);
        // *pointcloud_ptr = this->map_cloud;
        // open3d::visualization::DrawGeometries({pointcloud_ptr}, "graphSlam::update Pointcloud", 1920, 1080);
        // map_cloud.push_back(transformed.VoxelDownSample(voxel_size/4));
    }

    // if( export_map)
    // {

    // }
}
/**
 * @brief Construct a new key Frame::key Frame object
 *
 * @param id
 * @param cloud
 * @param odom
 */
// class keyFrame Registration
keyFrame::keyFrame()
{

}

keyFrame::keyFrame(int _id, open3d::geometry::PointCloud _cloud,
                    Eigen::Matrix4d odom)
{
    std::cout << "keyFrame initialized" << std::endl;
    id = _id;
    cloud = _cloud;
    node = open3d::pipelines::registration::PoseGraphNode(odom);
}

keyFrame::~keyFrame()
{
    std::cout << "keyFrame delete" << std::endl;
}
