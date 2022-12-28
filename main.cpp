
// pandas data frame
#include <DataFrame/DataFrame.h>                  // Main DataFrame header
#include <DataFrame/DataFrameFinancialVisitors.h> // Financial algorithms
#include <DataFrame/DataFrameMLVisitors.h>        // Machine-learning algorithms
#include <DataFrame/DataFrameStatsVisitors.h>     // Statistical algorithms
#include <DataFrame/DataFrameTypes.h>             //
#include <DataFrame/Utils/DateTime.h>             // Cool and handy date-time object
// open3d

#include <stdio.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "graphSlam.h"
#include "open3d/Open3D.h"
#include "open3d/pipelines/registration/GlobalOptimization.h"
#include "open3d/pipelines/registration/PoseGraph.h"

// A DataFrame with string index type
//
// using MyDataFrame =
// hmdf::StdDataFrame<std::vector<std::vector<std::vector<double>>>>;
using MyDataFrame = hmdf::StdDataFrame<std::string>;

/**
 * @brief 
 * 
 * @param file_path 
 * @param input_df 
 * @param cut 
 * @param output 
 */
int read_cloud(const char *file_path,
                bool input_df,
                bool cut,
                open3d::geometry::PointCloud *cloud_output)
{
    MyDataFrame ibm_df;
    open3d::geometry::PointCloud geom;
    bool ret = false;

    if (file_path == NULL)
        return -1;

    std::cout << "Reading " << file_path << std::endl;

    if (input_df == false)
        ret = ibm_df.read((const char *)file_path, hmdf::io_format::csv2);

    if (ret == false)
    {
        std::cout << "Error reading" << file_path << std::endl;
        return -1;
    }
    if (cut)
    {
        // ibm_df = ibm_df;
        // remove values less than threshold
    }
    try
    {
        ibm_df.read((const char *)file_path, hmdf::io_format::csv2);
        if (ibm_df.empty() == true)
            return -1;

        std::vector<double> close_const_ref =
            ibm_df.get_column<double>("IBM_Close");
        std::cout << "close_const_ref " << close_const_ref.at(2) << std::endl;
        std::cout << "close_const_ref size" << close_const_ref.size() << std::endl;
    }
    catch (const hmdf::DataFrameError &ex)
    {
        std::cout << ex.what() << std::endl;
    }
    // get output
    *cloud_output = geom;
    // open3d::geometry::Geometry3D::TranslatePoints();
    Eigen::Matrix4d_u node;
    // open3d::pipelines::registration::PoseGraphNode(&node);
}

/**
 * @brief 
 * 
 * @param graph_slam 
 */
void update_optimization(graphSlam graph_slam)
{
    graph_slam.map_cloud = open3d::geometry::PointCloud();
    for (size_t point_id = 0; point_id < graph_slam.keyframes.size();
        point_id++)
    {
        graph_slam.keyframes[point_id].cloud.Transform(
            graph_slam.graph.nodes_[point_id].pose_);
        Eigen::Vector3d color(0.9375, 0.3125, 0.546875);
        graph_slam.keyframes[point_id].cloud.PaintUniformColor(color);
        graph_slam.map_cloud += graph_slam.keyframes[point_id].cloud;
    }
}

/**
 * @brief 
 * 
 * @param graph_slam 
 * @param voxel_size 
 */
void optimize_pose_graph(graphSlam graph_slam, double voxel_size)
{
    double max_correspondence_distance = 0;
    double edge_prune_threshold = 0;
    double reference_node = 0;
    int preference_loop_closure = 0;
    open3d::pipelines::registration::GlobalOptimizationOption option(
        max_correspondence_distance = voxel_size * 1.4,
        edge_prune_threshold = 0.25, reference_node = 0,
        preference_loop_closure = 5);
    std::shared_ptr<open3d::pipelines::registration::PoseGraph>
        pose_graph_input;
    open3d::pipelines::registration::GlobalOptimizationConvergenceCriteria
        criteria;
    open3d::pipelines::registration::GlobalOptimizationLevenbergMarquardt
        optimization_method;
}

/**
 * @brief 
 * 
 * @param pcd 
 * @param voxel_size 
 * @return std::tuple<std::shared_ptr<open3d::geometry::PointCloud>,
 * std::shared_ptr<open3d::pipelines::registration::Feature>> 
 */
std::tuple<std::shared_ptr<open3d::geometry::PointCloud>,
            std::shared_ptr<open3d::pipelines::registration::Feature>>
preprocess_point_cloud(open3d::geometry::PointCloud &pcd,
                        const float voxel_size)
{
    auto pcd_down = pcd.VoxelDownSample(voxel_size);
    pcd_down->EstimateNormals(
        open3d::geometry::KDTreeSearchParamHybrid(2 * voxel_size, 30));
    auto pcd_fpfh = open3d::pipelines::registration::ComputeFPFHFeature(
        *pcd_down,
        open3d::geometry::KDTreeSearchParamHybrid(5 * voxel_size, 100));
    return std::make_tuple(pcd_down, pcd_fpfh);
}

/**
 * @brief 
 * 
 * @param source 
 * @param target 
 * @param voxel_size 
 * @return std::tuple<
 * std::shared_ptr<open3d::geometry::PointCloud>,
 * std::shared_ptr<open3d::geometry::PointCloud>,
 * std::shared_ptr<open3d::pipelines::registration::Feature>,
 * std::shared_ptr<open3d::pipelines::registration::Feature>> 
 */
std::tuple<
    std::shared_ptr<open3d::geometry::PointCloud>,
    std::shared_ptr<open3d::geometry::PointCloud>,
    std::shared_ptr<open3d::pipelines::registration::Feature>,
    std::shared_ptr<open3d::pipelines::registration::Feature>>
prepare_dataset(open3d::geometry::PointCloud &source,
                open3d::geometry::PointCloud &target,
                double voxel_size)
{
    //     std::shared_ptr<open3d::geometry::PointCloud> source_transformed_ptr(new open3d::geometry::PointCloud);
    //     std::shared_ptr<open3d::geometry::PointCloud> target_ptr(new open3d::geometry::PointCloud);
    //     *source_transformed_ptr = source;
    //     *target_ptr = target;
    const Eigen::Matrix4d &M = Eigen::Matrix4d::Identity();
    source.Transform(M);

    // Prepare input
    std::shared_ptr<open3d::geometry::PointCloud> source_down, target_down;
    std::shared_ptr<open3d::pipelines::registration::Feature> source_fpfh, target_fpfh;
    std::tie(source_down, source_fpfh) = preprocess_point_cloud(source, voxel_size);
    std::tie(target_down, target_fpfh) = preprocess_point_cloud(target, voxel_size);

    return std::make_tuple(source_down, target_down, source_fpfh, target_fpfh);
}

/**
 * @brief 
 * 
 * @param source 
 * @param target 
 * @param source_feature 
 * @param target_feature 
 * @param voxel_size 
 * @return open3d::pipelines::registration::RegistrationResult 
 */
open3d::pipelines::registration::RegistrationResult execute_global_registration(
    std::shared_ptr<open3d::geometry::PointCloud> source,
    std::shared_ptr<open3d::geometry::PointCloud> target,
    std::shared_ptr<open3d::pipelines::registration::Feature> source_feature,
    std::shared_ptr<open3d::pipelines::registration::Feature> target_feature,
    double voxel_size)
{
    double distance_threshold = voxel_size * 1.5;
    std::vector<std::reference_wrapper<
        const open3d::pipelines::registration::CorrespondenceChecker>>
        correspondence_checker;
    auto correspondence_checker_edge_length = open3d::pipelines::registration::
        CorrespondenceCheckerBasedOnEdgeLength(0.9);
    auto correspondence_checker_distance = open3d::pipelines::registration::
        CorrespondenceCheckerBasedOnDistance(distance_threshold);

    correspondence_checker.push_back(correspondence_checker_edge_length);
    correspondence_checker.push_back(correspondence_checker_distance);
    open3d::pipelines::registration::RegistrationResult result = open3d::
        pipelines::registration::RegistrationRANSACBasedOnFeatureMatching(
            *source, *target, *source_feature, *target_feature, true,
            distance_threshold,
            open3d::pipelines::registration::TransformationEstimationPointToPoint(false),
            3, // default transformation
            correspondence_checker);

    return result;
}
/**
 * @brief 
 * 
 * @param source 
 * @param target 
 * @param source_fpfh 
 * @param target_fpfh 
 * @param voxel_size 
 * @param result_ransac 
 * @return open3d::pipelines::registration::RegistrationResult 
 */
open3d::pipelines::registration::RegistrationResult refine_registration(
    open3d::geometry::PointCloud source,
    open3d::geometry::PointCloud target,
    std::shared_ptr<open3d::pipelines::registration::Feature>  source_fpfh,
    std::shared_ptr<open3d::pipelines::registration::Feature>  target_fpfh,
    double voxel_size,
    open3d::pipelines::registration::RegistrationResult result_ransac)
{
    double distance_threshold = voxel_size * 0.4;
    source.EstimateNormals();
    target.EstimateNormals();
    open3d::pipelines::registration::RegistrationResult result =
        open3d::pipelines::registration::RegistrationICP(
            source, target, distance_threshold,
            result_ransac.transformation_,
            open3d::pipelines::registration::TransformationEstimationPointToPlane());
    return result;
}

/**
 * @brief 
 * 
 * @param cloud1 
 * @param cloud2 
 * @param voxel_size 
 * @return open3d::pipelines::registration::RegistrationResult 
 */
open3d::pipelines::registration::RegistrationResult 
match(open3d::geometry::PointCloud &cloud1,
        open3d::geometry::PointCloud &cloud2,
        double voxel_size)
{
    std::shared_ptr<open3d::geometry::PointCloud> source_down, target_down;
    std::shared_ptr<open3d::pipelines::registration::Feature> source_fpfh, target_fpfh;
    std::tie(source_down, target_down, source_fpfh, target_fpfh) = prepare_dataset(cloud1, cloud2, voxel_size);

    open3d::pipelines::registration::RegistrationResult result_ransac =
        execute_global_registration(source_down, target_down,
                                source_fpfh, target_fpfh,
                                voxel_size);

    open3d::pipelines::registration::RegistrationResult result_icp = refine_registration(cloud1, cloud2, 
                                                                                        source_fpfh, target_fpfh,
                                                                                        voxel_size, result_ransac);
    return result_icp;
}

/**
 * @brief 
 * 
 * @param input_cloud 
 */
void relocate(open3d::geometry::PointCloud &input_cloud)
{
    std::string dataset_path = "output/transformed";

}

int main(void)
{
    std::cout << "\nSLAM START ..." << std::endl;
    double voxel_size = 1000;
    bool visualize = true;
    bool export_map = false;
    open3d::geometry::PointCloud  input_cloud;
    std::string dataset_path = "../5/pcap_csv/";
    int ret = read_cloud("/home/tien.hoang/Desktop/mini_hdd/test_data/5/pcap_csv/pcap_out_000017.csv", false, true, &input_cloud);

    // cloud_files = sorted([dataset_path + x for x in os.listdir(dataset_path) if '.csv' in x])[:5]

    graphSlam graph_slam;
    std::cout << "\nRead sample data" << std::endl;
    // open3d_gemo gemo_test;
    // read_cloud("/home/vm/Desktop/lib/DataFrame/data/IBM.csv", false, false,
    //            &gemo_test);
}