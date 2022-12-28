#ifndef _GRAPH_SLAM_H_// #GRAPH_SLAM_H 
#define _GRAPH_SLAM_H_

#include <iostream>
#include <vector>
#include <string>
#include <stdbool.h>
#include "open3d/Open3D.h"
#include "open3d/pipelines/registration/GlobalOptimization.h"
#include "open3d/pipelines/registration/PoseGraph.h"


// class keyFrame;


class keyFrame
{
private:
    /* data */    
public:
    keyFrame(int id,open3d::geometry::PointCloud cloud,  Eigen::Matrix4d odom);
    ~keyFrame();

    // data
    int id;
    Eigen::Matrix4d odom;
    open3d::geometry::PointCloud cloud;
    open3d::pipelines::registration::PoseGraphNode node;
};


class graphSlam
{
private:
    /* data */
    std::vector<int> points;
public:
    graphSlam(/* args */);
    ~graphSlam();
    void update(open3d::geometry::PointCloud &cloud, 
                double voxel_size=1000, 
                bool export_map=false, 
                bool visualize = false);
    // data
    open3d::pipelines::registration::PoseGraph graph;
    open3d::geometry::PointCloud map_cloud;
    std::vector<keyFrame> keyframes;
};



#endif /*_GRAPH_SLAM_H_*/