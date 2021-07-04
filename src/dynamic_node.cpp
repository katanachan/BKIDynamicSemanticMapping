#include <string>
#include <iostream>
#include <ros/ros.h>

#include "bkioctomap.h"
#include "markerarray_pub.h"

static void load_pcd(std::string filename, semantic_bki::point3f &origin,
                 semantic_bki::PCLPointCloud &cloud) {
    pcl::PCLPointCloud2 cloud2;
    //semantic_bki::PCLPointCloud rot_cloud;
    Eigen::Vector4f _origin;
    //Eigen::Vector3f _zero_origin = Eigen::Matrix< float, 3, 1 >::Zero();
    Eigen::Quaternionf orientation;
    pcl::io::loadPCDFile(filename, cloud2, _origin, orientation);
    pcl::fromPCLPointCloud2(cloud2, cloud);
    // _zero_origin[0] = _origin[0];
    // _zero_origin[1] = _origin[1];
    // _zero_origin[2] = _origin[2];
    //orientation = orientation.conjugate();
    //pcl::transformPointCloud(rot_cloud, cloud, _zero_origin, orientation);
    
    origin.x() = _origin[0];
    origin.y() = _origin[1];
    origin.z() = _origin[2];
}

static void load_pcd(std::string filename, semantic_bki::point3f &origin, Eigen::Quaternionf &orientation,
                 semantic_bki::PCLPointCloud &cloud) {
    pcl::PCLPointCloud2 cloud2;
    //semantic_bki::PCLPointCloud rot_cloud;
    Eigen::Vector4f _origin;
    //Eigen::Vector3f _zero_origin = Eigen::Matrix< float, 3, 1 >::Zero();
    pcl::io::loadPCDFile(filename, cloud2, _origin, orientation);
    pcl::fromPCLPointCloud2(cloud2, cloud);
    // _zero_origin[0] = _origin[0];
    // _zero_origin[1] = _origin[1];
    // _zero_origin[2] = _origin[2];
    //orientation = orientation.conjugate();
    //pcl::transformPointCloud(rot_cloud, cloud, _zero_origin, orientation);
    
    origin.x() = _origin[0];
    origin.y() = _origin[1];
    origin.z() = _origin[2];
}

    /*
    *\brief Check if the flow computation is valid between scans ie.
    * if the viewpoint has shifted beyond a threshold, discard the reading.
    * @ param origin_prev : robot's position in the previous scan
    * @ param origin_now : robot's position in the current scan
    */
    static bool is_data_valid(const semantic_bki::point3f &origin_prev, 
                    const semantic_bki::point3f &origin_now){
            //std::cout << "Displacement between scans" << origin_now - origin_prev << std::endl;
            if ((origin_prev - origin_now).norm() > 0.5) 
                return false;
            else
                return true;
    }

    static bool is_rotation_valid(const Eigen::Quaternionf &prev,
                const Eigen::Quaternionf &next){
            std::cout << prev.angularDistance(next) << std::endl;
            if (prev.angularDistance(next) > 0.05 ) // 2.86 deg tolerance
                return false;
            else
                return true;

    }

static void publish_map(semantic_bki::MarkerArrayPub &m_pub, semantic_bki::MarkerArrayPub &v_pub,
        const semantic_bki::SemanticBKIOctoMap &map, const semantic_bki::point3f &origin,
        const int num_class = 4){

    m_pub.clear_map(0);
    v_pub.clear_map(0);

    float max_var = std::numeric_limits<float>::min();
    float min_var = std::numeric_limits<float>::max(); 
    for (auto it = map.begin_leaf(); it != map.end_leaf(); ++it) {
        if (it.get_node().get_state() == semantic_bki::State::OCCUPIED) {
            semantic_bki::point3f p = it.get_loc(); //get octree node
            int semantics = it.get_node().get_semantics(); // get associated semantics
            m_pub.insert_point3d_semantics(p.x(), p.y(), p.z(), it.get_size(), 
                            semantics, 0); //data -> marker
            std::vector<float> vars(num_class);
            it.get_node().get_vars(vars);
            v_pub.insert_point3d_variance(p.x(), p.y(), p.z(), min_var, max_var,
                                         it.get_size(), vars[semantics]);
            if (vars[semantics] > max_var)
		          max_var = vars[semantics];
		        if (vars[semantics] < min_var)
		          min_var = vars[semantics];
        }
    }
    //m_pub.insert_point3d_semantics(origin.x(), origin.y(), origin.z(), 0.2 , 4, 0);
    m_pub.publish();
    v_pub.publish();
    std::cout << "max_var: " << max_var << std::endl;
    std::cout << "min_var: " << min_var << std::endl;

}

static void run_map_experiment(semantic_bki::SemanticBKIOctoMap &map, ros::NodeHandle &nh,
                            const semantic_bki::PCParams *train_params,
                            const semantic_bki::MapParams *mparams,
                            const int scan_num, std::string file_prefix ){

    semantic_bki::point3f prev_origin;
    semantic_bki::PCLPointCloud prev_cloud;
    Eigen::Quaternionf prev_rot;
    std::string prev_filename(file_prefix +  "_1.pcd");
    semantic_bki::MarkerArrayPub m_pub(nh, "/semantic_bki", mparams->resolution);
    semantic_bki::MarkerArrayPub v_pub(nh, "/semantic_bki_variance", mparams->resolution);
    load_pcd(prev_filename, prev_origin, prev_rot, prev_cloud);

    for (int scan_id = 2; scan_id <= scan_num; ++scan_id) {
        semantic_bki::PCLPointCloud cloud;
        semantic_bki::point3f origin;
        Eigen::Quaternionf rot;
        std::string filename(file_prefix +  "_" + std::to_string(scan_id) + ".pcd");
        load_pcd(filename, origin, rot, cloud);
        if (is_rotation_valid(rot, prev_rot) && is_data_valid(origin, prev_origin)){
        //if (is_data_valid(origin, prev_origin)){
        //if (true){
            map.insert_pointcloud(prev_cloud, prev_origin, (origin - prev_origin), 
                                train_params, (semantic_bki::ScanStep) scan_id);
            ROS_INFO_STREAM("Scan " << scan_id - 1 << " done");
            publish_map(m_pub, v_pub, map, prev_origin, mparams->num_classes); // we only need to 
            //publish if the map is updated
        }
        else
            ROS_INFO_STREAM("Scan " << scan_id - 1 << " discarded");
	    prev_origin = origin;
        prev_cloud = cloud; //transfer loaded cloud as previous cloud to be inserted in later
        prev_rot = rot;  

    }


}

int main(int argc, char **argv) {
    ros::init(argc, argv, "dynamic_node");
    ros::NodeHandle nh("~");
    
    std::string map_topic_csm("/semantic_csm");
    std::string map_topic("/semantic_bki");
    std::string var_topic_csm("/semantic_csm_variance");
    std::string var_topic("/semantic_bki_variance");
    std::string dir;
    std::string prefix;

    int scan_num = 0;
    bool spatial = false;
    
    semantic_bki::PCParams *train_params = new semantic_bki::PCParams{0.1, 0.5, -1}; //ds_resolution, free_resolution, max_range
    semantic_bki::MapParams *mparams = new semantic_bki::MapParams{
                                    0.1, 4, 2, //resolution, block_depth, num_classes 
                                    
                                    1.0, 1.0, 1.0f, //sf2, ell, prior 

                                    0.2, 0.2, // flow_sf2, flow_ell

                                    1.0f, 0.3, 0.7, //var_thresh, free_thresh, occupied_thresh

                                    false, //spatiotemporal

                                    {0, 2} // only dynamic class the robot
                                    };


    nh.param<std::string>("dir", dir, dir);
    nh.param<std::string>("prefix", prefix, prefix);
    nh.param<int>("block_depth", mparams->block_depth, mparams->block_depth);

    nh.param<double>("sf2", mparams->sf2, mparams->sf2);
    nh.param<double>("ell", mparams->ell, mparams->ell);
    nh.param<float>("prior", mparams->prior, mparams->prior);

    nh.param<float>("flow_sf2", mparams->flow_sf2, mparams->flow_sf2);
    nh.param<float>("flow_ell", mparams->flow_ell, mparams->flow_ell);
    
    
    nh.param<float>("var_thresh", mparams->var_thresh, mparams->var_thresh);
    nh.param<double>("free_thresh", mparams->free_thresh, mparams->free_thresh);
    nh.param<double>("occupied_thresh", mparams->occupied_thresh, mparams->occupied_thresh);
    
    nh.param<double>("resolution", mparams->resolution, mparams->resolution);
    nh.param<int>("num_class", mparams->num_classes, mparams->num_classes);
    nh.param<double>("free_resolution", train_params->free_resolution, train_params->free_resolution);
    nh.param<double>("ds_resolution", train_params->ds_resolution, train_params->ds_resolution);
    nh.param<int>("scan_num", scan_num, scan_num);
    nh.param<double>("max_range", train_params->max_range, train_params->max_range);

    nh.param<bool>("show_spatial", spatial, spatial);
    nh.getParam("moving_classes", mparams->dynamic);
   
    ROS_INFO_STREAM("Parameters:" << std::endl <<
            "dir: " << dir << std::endl <<
            "prefix: " << prefix << std::endl <<
            "block_depth: " << mparams->block_depth << std::endl <<
            "sf2: " << mparams->sf2 << std::endl <<
            "ell: " << mparams->ell << std::endl <<
            "prior: " << mparams->prior << std::endl <<
            "var_thresh: " << mparams->var_thresh << std::endl <<
            "free_thresh: " << mparams->free_thresh << std::endl <<
            "occupied_thresh: " << mparams->occupied_thresh << std::endl <<
            "resolution: " << mparams->resolution << std::endl <<
            "num_class: " << mparams->num_classes << std::endl <<
            "free_resolution: " << train_params->free_resolution << std::endl <<
            "ds_resolution: " << train_params->ds_resolution << std::endl <<
            "scan_sum: " << scan_num << std::endl <<
            "max_range: " << train_params->max_range
            );

    ros::Time start, end;

    // /////////////////////// Semantic CSM //////////////////////

    // semantic_bki::SemanticBKIOctoMap map_csm(mparams->resolution, 1, mparams->num_classes, 
    //             mparams->sf2, mparams->ell, mparams->prior, mparams->flow_sf2, mparams->flow_ell, 
    //             mparams->var_thresh, mparams->free_thresh, mparams->occupied_thresh, false); //vanilla map
    // ros::Time start = ros::Time::now();
    // semantic_bki::point3f origin_prev;
    // semantic_bki::PCLPointCloud cloud_prev;
    // Eigen::Quaternionf rot_prev;
    // std::string filename(dir + "/" + prefix + "_1.pcd");
    // load_pcd(filename, origin_prev, rot_prev, cloud_prev);
    // for (int scan_id = 2; scan_id <= scan_num; ++scan_id) {
    //     semantic_bki::PCLPointCloud cloud;
    //     semantic_bki::point3f origin;
    //     Eigen::Quaternionf rot;
    //     std::string filename(dir + "/" + prefix + "_" + std::to_string(scan_id) + ".pcd");
    //     load_pcd(filename, origin, rot, cloud); // loaded the next point cloud
    //     if (is_data_valid(origin, origin_prev)){
    //         map_csm.insert_pointcloud_csm(cloud_prev, origin_prev, train_params, (semantic_bki::ScanStep) scan_id);
    //         ROS_INFO_STREAM("Scan " << scan_id - 1 << " done");
    //     }
    //     else{
    //         ROS_INFO_STREAM("Scan " << scan_id - 1 << " discarded");
    //     }
	//     origin_prev = origin;
    //     cloud_prev = cloud; //transfer loaded cloud as previous cloud to be inserted in later
    //     rot_prev = rot;


    // }
    
    // //map_csm.sync_block((semantic_bki::ScanStep) (scan_num + 1));
    // ros::Time end = ros::Time::now(); 
    // ROS_INFO_STREAM("Semantic CSM finished in " << (end - start).toSec() << "s");

    // semantic_bki::MarkerArrayPub m_pub_csm(nh, map_topic_csm, mparams->resolution);
    // semantic_bki::MarkerArrayPub v_pub_csm(nh, var_topic_csm, mparams->resolution);
    // publish_map(m_pub_csm, v_pub_csm, map_csm);
    
    /////////////////////// Semantic BKI //////////////////////
    if (spatial){
        semantic_bki::SemanticBKIOctoMap map(mparams); //inferred map
 
        start = ros::Time::now();
        run_map_experiment(map, nh, train_params, mparams, scan_num, dir + "/" + prefix);
        //map.sync_block((semantic_bki::ScanStep) (scan_num + 1));
        end = ros::Time::now();
        ROS_INFO_STREAM("Semantic BKI finished in " << (end - start).toSec() << "s");
    }


    /////////////////////// Dynamic BKI //////////////////////
    mparams->spatiotemporal = true;
    semantic_bki::SemanticBKIOctoMap dmap(mparams); //inferred map
    start = ros::Time::now();
    
    run_map_experiment(dmap, nh, train_params, mparams, scan_num, dir + "/" + prefix);

    //map.sync_block((semantic_bki::ScanStep) (scan_num + 1));
    end = ros::Time::now();
    ROS_INFO_STREAM("Dynamic BKI finished in " << (end - start).toSec() << "s");


    delete mparams;
    delete train_params;

    ros::spin();

    return 0;
}
