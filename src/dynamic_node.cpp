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
        semantic_bki::SemanticBKIOctoMap &map, int num_class = 4){

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
            v_pub.insert_point3d_variance(p.x(), p.y(), p.z(), min_var, max_var, it.get_size(), vars[semantics]);
            if (vars[semantics] > max_var)
		          max_var = vars[semantics];
		        if (vars[semantics] < min_var)
		          min_var = vars[semantics];
        }
    }
    m_pub.publish();
    v_pub.publish();
    std::cout << "max_var: " << max_var << std::endl;
    std::cout << "min_var: " << min_var << std::endl;

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
    int block_depth = 4;
    double sf2 = 1.0;
    double ell = 1.0;
    float flow_sf2 = 0.2;
    float flow_ell = 0.2;
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
    semantic_bki::PCParams params = {0.1, 0.5, -1};
    semantic_bki::MapParams *mparams = new semantic_bki::MapParams{0.1, 4, 2, //resolution, block_depth, num_classes 
                                    1.0, 1.0, 1.0f, //sf2, ell, prior 

                                    0.2, 0.2, // flow_sf2, flow_ell

                                    1.0f, 0.3, 0.7, //var_thresh, free_thresh, occupied_thresh

                                    false //spatiotemporal
                                    };


    nh.param<std::string>("dir", dir, dir);
    nh.param<std::string>("prefix", prefix, prefix);
    nh.param<int>("block_depth", block_depth, block_depth);
    nh.param<double>("sf2", sf2, sf2);
    nh.param<double>("ell", ell, ell);
    nh.param<float>("flow_sf2", flow_sf2, flow_sf2);
    nh.param<float>("flow_ell", flow_ell, flow_ell);
    nh.param<float>("prior", prior, prior);
    nh.param<float>("var_thresh", var_thresh, var_thresh);
    nh.param<double>("free_thresh", free_thresh, free_thresh);
    nh.param<double>("occupied_thresh", occupied_thresh, occupied_thresh);
    nh.param<double>("resolution", resolution, resolution);
    nh.param<int>("num_class", num_class, num_class);
    nh.param<double>("free_resolution", params.free_resolution, params.free_resolution);
    nh.param<double>("ds_resolution", params.ds_resolution, params.ds_resolution);
    nh.param<int>("scan_num", scan_num, scan_num);
    nh.param<double>("max_range", params.max_range, params.max_range);
   
    ROS_INFO_STREAM("Parameters:" << std::endl <<
            "dir: " << dir << std::endl <<
            "prefix: " << prefix << std::endl <<
            "block_depth: " << block_depth << std::endl <<
            "sf2: " << sf2 << std::endl <<
            "ell: " << ell << std::endl <<
            "prior: " << prior << std::endl <<
            "var_thresh: " << var_thresh << std::endl <<
            "free_thresh: " << free_thresh << std::endl <<
            "occupied_thresh: " << occupied_thresh << std::endl <<
            "resolution: " << resolution << std::endl <<
            "num_class: " << num_class << std::endl <<
            "free_resolution: " << params.free_resolution << std::endl <<
            "ds_resolution: " << params.ds_resolution << std::endl <<
            "scan_sum: " << scan_num << std::endl <<
            "max_range: " << params.max_range
            );

    /////////////////////// Semantic CSM //////////////////////
    semantic_bki::SemanticBKIOctoMap map_csm(resolution, 1, num_class, 
                sf2, ell, prior, flow_sf2, flow_ell, var_thresh, free_thresh, occupied_thresh, false); //vanilla map
    ros::Time start = ros::Time::now();
    semantic_bki::point3f origin_prev;
    semantic_bki::PCLPointCloud cloud_prev;
    Eigen::Quaternionf rot_prev;
    std::string filename(dir + "/" + prefix + "_1.pcd");
    load_pcd(filename, origin_prev, rot_prev, cloud_prev);
    for (int scan_id = 2; scan_id <= scan_num; ++scan_id) {
        semantic_bki::PCLPointCloud cloud;
        semantic_bki::point3f origin;
        Eigen::Quaternionf rot;
        std::string filename(dir + "/" + prefix + "_" + std::to_string(scan_id) + ".pcd");
        load_pcd(filename, origin, rot, cloud); // loaded the next point cloud
        if (is_data_valid(origin, origin_prev) & is_rotation_valid(rot, rot_prev )){
            map_csm.insert_pointcloud_csm(cloud_prev, origin_prev, resolution, free_resolution, 
                    max_range, (semantic_bki::ScanStep) scan_id);
            ROS_INFO_STREAM("Scan " << scan_id - 1 << " done");
        }
        else{
            ROS_INFO_STREAM("Scan " << scan_id - 1 << " discarded");
        }
	    origin_prev = origin;
        cloud_prev = cloud; //transfer loaded cloud as previous cloud to be inserted in later
        rot_prev = rot;


    }
    
    //map_csm.sync_block((semantic_bki::ScanStep) (scan_num + 1));
    ros::Time end = ros::Time::now(); 
    ROS_INFO_STREAM("Semantic CSM finished in " << (end - start).toSec() << "s");

    semantic_bki::MarkerArrayPub m_pub_csm(nh, map_topic_csm, resolution);
    semantic_bki::MarkerArrayPub v_pub_csm(nh, var_topic_csm, resolution);
    publish_map(m_pub_csm, v_pub_csm, map_csm);
    
    /////////////////////// Semantic BKI //////////////////////
    semantic_bki::SemanticBKIOctoMap map(resolution, block_depth, num_class, sf2,
                 ell, prior, flow_sf2, flow_ell, var_thresh, free_thresh, occupied_thresh, false); //inferred map
    start = ros::Time::now();
    semantic_bki::point3f prev_origin_bki;
    semantic_bki::PCLPointCloud prev_cloud_bki;
    Eigen::Quaternionf prev_rot_bki;
    std::string prev_filename(dir + "/" + prefix + "_1.pcd");
    load_pcd(prev_filename, prev_origin_bki, prev_rot_bki, prev_cloud_bki);
    for (int scan_id = 2; scan_id <= scan_num; ++scan_id) {
        semantic_bki::PCLPointCloud cloud;
        semantic_bki::point3f origin;
        Eigen::Quaternionf rot;
        std::string filename(dir + "/" + prefix + "_" + std::to_string(scan_id) + ".pcd");
        load_pcd(filename, origin, rot, cloud);
        if (is_rotation_valid(rot, prev_rot_bki) && is_data_valid(origin, prev_origin_bki)){
            map.insert_pointcloud(prev_cloud_bki, prev_origin_bki, resolution, free_resolution,
                         max_range, (semantic_bki::ScanStep) scan_id);
            ROS_INFO_STREAM("Scan " << scan_id - 1 << " done");
        }
        else
            ROS_INFO_STREAM("Scan " << scan_id - 1 << " discarded");
	    prev_origin_bki = origin;
        prev_cloud_bki = cloud; //transfer loaded cloud as previous cloud to be inserted in later
        prev_rot_bki = rot;

        semantic_bki::MarkerArrayPub m_pub(nh, map_topic, resolution);
        semantic_bki::MarkerArrayPub v_pub(nh, var_topic, resolution);
        publish_map(m_pub, v_pub, map);
    }
    //map.sync_block((semantic_bki::ScanStep) (scan_num + 1));
    end = ros::Time::now();
    ROS_INFO_STREAM("Semantic BKI finished in " << (end - start).toSec() << "s");
 


    delete mparams;

    ros::spin();

    return 0;
}
