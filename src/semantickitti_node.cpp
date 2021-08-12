#include <string>
#include <iostream>
#include <ros/ros.h>

#include "markerarray_pub.h"
#include "semantickitti_util.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "semantickitti_node");
    ros::NodeHandle nh("~");

    std::string map_topic("/occupied_cells_vis_array");
    int block_depth = 4;
    double sf2 = 1.0;
    double ell = 1.0;
    float prior = 1.0f;
    float var_thresh = 1.0f;
    double free_thresh = 0.3;
    double occupied_thresh = 0.7;
    double resolution = 0.1;
    int num_class = 2;
    double free_resolution = 0.5;
    double ds_resolution = 0.1;
    int scan_num = 0;
    double max_range = -1;
    int sequence_no = 4;
    semantic_bki::MapParams *mparams = new semantic_bki::MapParams{
                                    0.1, 4, 2, //resolution, block_depth, num_classes 
                                    
                                    1.0, 1.0, 1.0f, //sf2, ell, prior 

                                    0.2, 0.2, // flow_sf2, flow_ell

                                    1.0f, 0.3, 0.7, //var_thresh, free_thresh, occupied_thresh

                                    true, //spatiotemporal,

                                    {1, 2, 3, 4, 5, 6, 7, 8}, // dynamic classes

                                    true //usually assumes that we have sampled free space
                                    };
    semantic_bki::PCParams *train_params = new semantic_bki::PCParams{0.1, 0.5, -1};
    //ds_resolution, free_resolution, max_range
    bool spatial = false;

    // SemanticKITTI
    std::string dir;
    std::string input_data_prefix;
    std::string input_label_prefix;
    std::string lidar_pose_file;
    std::string gt_label_prefix;
    std::string evaluation_result_prefix;
    bool query = false;
    bool visualize = false;

    nh.param<int>("block_depth", mparams->block_depth, mparams->block_depth);
    nh.param<double>("sf2", mparams->sf2, mparams->sf2);
    nh.param<double>("ell", mparams->ell, mparams->ell);
    nh.param<float>("prior", mparams->prior, mparams->prior);

    nh.param<float>("flow_sf2", mparams->flow_sf2, mparams->flow_sf2);
    nh.param<float>("flow_ell", mparams->flow_ell, mparams->flow_ell);
    
    
    nh.param<float>("var_thresh", mparams->var_thresh, mparams->var_thresh);
    nh.param<double>("free_thresh", mparams->free_thresh, mparams->free_thresh);
    nh.param<double>("occupied_thresh", mparams->occupied_thresh, mparams->occupied_thresh);
    nh.param<bool>("spatiotemporal", mparams->spatiotemporal, mparams->spatiotemporal);
    nh.param<bool>("free_sample", mparams->free_sample, mparams->free_sample);

    
    nh.param<double>("resolution", mparams->resolution, mparams->resolution);
    nh.param<int>("num_class", mparams->num_classes, mparams->num_classes);
    nh.param<double>("free_resolution", train_params->free_resolution, train_params->free_resolution);
    nh.param<double>("ds_resolution", train_params->ds_resolution, train_params->ds_resolution);
    nh.param<int>("scan_num", scan_num, scan_num);
    nh.param<double>("max_range", train_params->max_range, train_params->max_range);

    nh.param<bool>("show_spatial", spatial, spatial);
    nh.param<int>("sequence_no", sequence_no, sequence_no);
    nh.getParam("moving_classes", mparams->dynamic);

    // SemanticKITTI
    nh.param<std::string>("dir", dir, dir);
    nh.param<std::string>("input_data_prefix", input_data_prefix, input_data_prefix);
    nh.param<std::string>("input_label_prefix", input_label_prefix, input_label_prefix);
    nh.param<std::string>("lidar_pose_file", lidar_pose_file, lidar_pose_file);
    nh.param<std::string>("gt_label_prefix", gt_label_prefix, gt_label_prefix);
    nh.param<std::string>("evaluation_result_prefix", evaluation_result_prefix, evaluation_result_prefix);
    nh.param<bool>("query", query, query);
    nh.param<bool>("visualize", visualize, visualize);

    ROS_INFO_STREAM("Parameters:" << '\n' <<
      "block_depth: " << block_depth << '\n' <<
      "sf2: " << mparams->sf2 << '\n' <<
      "ell: " << mparams->ell << '\n' <<
      "flow_sf2: " << mparams->flow_sf2 << '\n' <<
      "flow_ell: " << mparams->flow_ell << '\n' <<
      "prior: " << mparams->prior << '\n' <<
      "var_thresh: " << mparams->var_thresh << '\n' <<
      "free_thresh: " << mparams->free_thresh << '\n' <<
      "occupied_thresh: " << mparams->occupied_thresh << '\n' <<
      "resolution: " << mparams->resolution << '\n' <<
      "num_class: " << mparams->num_classes << '\n' <<
      "free_resolution: " << train_params->free_resolution << '\n' <<
      "ds_resolution: " << train_params->ds_resolution << '\n' <<
      "scan_sum: " << scan_num << '\n' <<
      "max_range: " << train_params->max_range << '\n' << 
      "spatiotemporal: " << mparams->spatiotemporal << '\n' << 
      "free_sample: " << mparams->free_sample << '\n' << 

      "SemanticKITTI:" << '\n' <<
      "dir: " << dir << '\n' <<
      "input_data_prefix: " << input_data_prefix << '\n' <<
      "input_label_prefix: " << input_label_prefix << '\n' <<
      "lidar_pose_file: " << lidar_pose_file << '\n' <<
      "gt_label_prefix: " << gt_label_prefix << '\n' <<
      "evaluation_result_prefix: " << evaluation_result_prefix << '\n' <<
      "query: " << query << '\n' <<
      "visualize:" << visualize
      );

    
    ///////// Build Map /////////////////////
    SemanticKITTIData semantic_kitti_data(nh, mparams, train_params, map_topic, sequence_no);
    semantic_kitti_data.read_lidar_poses(dir + '/' + lidar_pose_file);
    semantic_kitti_data.set_up_evaluation(dir + '/' + gt_label_prefix, dir + '/' + evaluation_result_prefix);
    semantic_kitti_data.process_scans(dir + '/' + input_data_prefix, dir + '/' + input_label_prefix, scan_num, query, visualize);

    delete mparams;
    delete train_params;

    ros::spin();
    return 0;
}
