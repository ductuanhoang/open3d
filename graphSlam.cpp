
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
extern open3d::pipelines::registration::RegistrationResult  match(open3d::geometry::PointCloud &cloud1,
            open3d::geometry::PointCloud &cloud2,
            double voxel_size);

graphSlam::graphSlam(/* args */)
{
    std::cout << "graphSlam initialized" << std::endl;
    map_cloud = open3d::geometry::PointCloud();
    graph = (open3d::pipelines::registration::PoseGraph());
}

graphSlam::~graphSlam()
{
    std::cout << "graphSlam deleted" << std::endl;
}

void graphSlam::update(open3d::geometry::PointCloud &cloud, 
                        double voxel_size, 
                        bool export_map,
                        bool visualize)
{
    if( keyframes.size() == 0)
    {
        // std::vector<open3d::geometry::PointCloud>  kf = new keyFrame(0, cloud, );
        Eigen::Matrix4d &M = Eigen::Matrix4d::Identity();
        keyFrame kf = keyFrame(0, cloud, M);
        keyframes.push_back(kf);
        graph.nodes_.push_back(keyframes[ keyframes.size() - 1].node);
    }
    else
    {
        open3d::pipelines::registration::RegistrationResult result_icp = match(cloud, keyframes[keyframes.size() - 1].cloud, voxel_size);
        Eigen::Matrix4d odom = result_icp.transformation_.dot(keyframes[keyframes.size() - 1].odom);
        keyFrame kf = keyFrame(keyframes.size(), cloud, odom);
        graph.nodes
    }
}

// class keyFrame Registration
keyFrame::keyFrame(int id, open3d::geometry::PointCloud cloud, 
                    Eigen::Matrix4d odom)
{
    std::cout << "keyFrame initialized" << std::endl;
    cloud = open3d::geometry::PointCloud();
    node = open3d::pipelines::registration::PoseGraphNode(odom);
}

keyFrame::~keyFrame()
{
    std::cout << "keyFrame delete" << std::endl;
}

