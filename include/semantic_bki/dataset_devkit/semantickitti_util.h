#pragma once

#include <fstream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include "flow_point_types.h"
#include "bkioctomap.h"

class SemanticKITTIData {
  public:
    SemanticKITTIData(ros::NodeHandle& nh,
             const semantic_bki::MapParams *map_params,
             const semantic_bki::PCParams *pc_train_params,
             const std::string &map_topic,
             const int sequence_id = 4
             )
        // SemanticKITTIData(ros::NodeHandle& nh,
        //      double resolution, double block_depth,
        //      double sf2, double ell,
        //      int num_class, double free_thresh,
        //      double occupied_thresh, float var_thresh, 
	      //      double ds_resolution,
        //      double free_resolution, double max_range,
        //      std::string map_topic,
        //      float prior)
      : nh_(nh)
      , resolution_(map_params->resolution)
      , train_params( new semantic_bki::PCParams)
      , sequence(sequence_id)
       {
        map_ = new semantic_bki::SemanticBKIOctoMap(map_params);
        // map_ = new semantic_bki::SemanticBKIOctoMap(resolution, block_depth, num_class,
        //                    sf2, ell, prior, var_thresh, free_thresh, occupied_thresh);
        m_pub_ = new semantic_bki::MarkerArrayPub(nh_, map_topic, resolution_);
      	init_trans_to_ground_ << 1, 0, 0, 0,
                                 0, 0, 1, 0,
                                 0,-1, 0, 1,
                                 0, 0, 0, 1;

        train_params->max_range = pc_train_params->max_range;
        train_params->ds_resolution = pc_train_params->ds_resolution;
        train_params->free_resolution = pc_train_params->free_resolution;

        init_calibration_matrix();
        init_remap_dynamics();
      }

    bool read_lidar_poses(const std::string &lidar_pose_name) {
      if (std::ifstream(lidar_pose_name)) {
        std::ifstream fPoses;
        fPoses.open(lidar_pose_name.c_str());
        while (!fPoses.eof()) {
          std::string s;
          std::getline(fPoses, s);
          if (!s.empty()) {
            std::stringstream ss;
            ss << s;
            Eigen::Matrix4d t_matrix = Eigen::Matrix4d::Identity();
            for (int i = 0; i < 3; ++i)
              for (int j = 0; j < 4; ++j)
                ss >> t_matrix(i, j);
            lidar_poses_.push_back(t_matrix);
          }
        }
        fPoses.close();
        return true;
        } else {
         ROS_ERROR_STREAM("Cannot open evaluation list file " << lidar_pose_name);
         return false;
      }
    }
    ~SemanticKITTIData(){
      delete map_;
      delete m_pub_;
    }

    bool process_scans(const std::string &input_data_dir, const std::string &input_label_dir, 
                        const int scan_num, const bool query, const bool visualize) {
      semantic_bki::point3f origin, prev_origin;
      //get the previous cloud, populate the previous origin with camera-to-lidar transform
      semantic_bki::PCLPointCloud::Ptr prev_cloud =
	            get_one_scan(input_data_dir, input_label_dir, 0, prev_origin);
      
      for (int scan_id  = 1; scan_id < scan_num; ++scan_id) {
	 
	      semantic_bki::PCLPointCloud::Ptr cloud = 
                get_one_scan(input_data_dir, input_label_dir, scan_id, origin);
	      map_->insert_pointcloud(*prev_cloud, prev_origin, (origin - prev_origin), 
                    this->train_params);
        std::cout << "Inserted point cloud at " << scan_id << std::endl;
        std::cout << "Displacement between scans is:" << (origin - prev_origin) << std::endl;
        //std::string file_name = input_data_dir + "semantic_kitti_" + std::to_string(scan_id) + ".pcd";
        //pcl::io::savePCDFileASCII(file_name, *cloud);
        if (query) {
          //for (int query_id = scan_id - 10; query_id >= 0 && query_id <= scan_id; ++query_id){
          query_scan(input_data_dir, scan_id - 1, true); //only querying present frame
          //note, you were working with prev_cloud, so queried scan should be for the previous cloud
        }

        if (visualize)
	        publish_map();
        
        prev_origin = origin;
	      // safe to reassign shared_ptr
        prev_cloud = cloud;
      
      }
      return 1;
    }

    void publish_map() {
      m_pub_->clear_map(resolution_);
      for (auto it = map_->begin_leaf(); it != map_->end_leaf(); ++it) {
        if (it.get_node().get_state() == semantic_bki::State::OCCUPIED) {
          semantic_bki::point3f p = it.get_loc();
          m_pub_->insert_point3d_semantics(p.x(), p.y(), p.z(), it.get_size(), 
                                      it.get_node().get_semantics(), 2);
        }
      }
      m_pub_->publish();
    }

    void set_up_evaluation(const std::string &gt_label_dir, const std::string &evaluation_result_dir) {
      gt_label_dir_ = gt_label_dir;
      evaluation_result_dir_ = evaluation_result_dir;
    }

    void query_scan(const std::string &input_data_dir, const int scan_id, const bool get_vels) {
      char scan_id_c[256];
      sprintf(scan_id_c, "%06d", scan_id);
      std::string scan_name = input_data_dir + std::string(scan_id_c) + ".bin";
      std::string gt_name = gt_label_dir_ + std::string(scan_id_c) + ".label";
      std::string result_name = evaluation_result_dir_ + std::string(scan_id_c) + ".txt";
      std::string velocity_name = evaluation_result_dir_ + std::string(scan_id_c) + "flow.txt";
      pcl::PointCloud<pcl::PointXYZL>::Ptr cloud = kitti2pcl(scan_name, gt_name);
      Eigen::Matrix4d transform = lidar_poses_[scan_id];
      
      Eigen::Matrix4d new_transform = init_trans_to_ground_ * transform * calibration;
      pcl::transformPointCloud (*cloud, *cloud, new_transform);
      if (get_vels)
        std::cout << "writing to flow file" << std::endl;

      std::ofstream result_file;
      std::ofstream flow_file;
      result_file.open(result_name);
      if (get_vels)
        flow_file.open(velocity_name);
      for (int i = 0; i < cloud->points.size(); ++i) {
        semantic_bki::SemanticOcTreeNode node = map_->search(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
        int pred_label = 0;
        int gt_ll = cloud->points[i].label;
	      if (node.get_state() == semantic_bki::State::OCCUPIED)
	        pred_label = node.get_semantics();
        result_file << gt_ll << " " << pred_label << "\n";
        if (get_vels){
          if (gt_ll > 19) //hardcoded
            gt_ll = remap_dynamics.find(gt_ll)->second;
          flow_file << node.get_flow(pred_label) << " " << node.get_flow(gt_ll) << "\n"; 
        }
      }
      result_file.close();
      if (get_vels)
        flow_file.close();
    }

  
  private:
    ros::NodeHandle nh_;
    double resolution_;
    semantic_bki::PCParams *train_params;
    semantic_bki::SemanticBKIOctoMap* map_;
    const int sequence;
    semantic_bki::MarkerArrayPub* m_pub_;
    ros::Publisher color_octomap_publisher_;
    tf::TransformListener listener_;
    std::ofstream pose_file_;
    std::vector<Eigen::Matrix4d> lidar_poses_;
    std::string gt_label_dir_;
    std::string evaluation_result_dir_;
    Eigen::Matrix4d init_trans_to_ground_;
    Eigen::Matrix4d calibration;
    std::map<int, int> remap_dynamics;

    void init_remap_dynamics(){
      remap_dynamics[20] = 1;
      remap_dynamics[21] = 7;
      remap_dynamics[22] = 6;
      remap_dynamics[23] = 8;
      remap_dynamics[24] = 5;
      remap_dynamics[25] = 4;
    }

    int check_element_in_vector(const long long element, const std::vector<long long>& vec_check) {
      for (int i = 0; i < vec_check.size(); ++i)
        if (element == vec_check[i])
          return i;
      return -1;
    }

    void init_calibration_matrix(){
      std::cout << "Initializing calibration matrix for sequence " << sequence <<std::endl;
      if (sequence == 0 || sequence == 1 || sequence == 2 || sequence >= 13)
      
        // 00-02: 2011_10_03_drive
        calibration << 0.000427680238558, -0.999967248494602, -0.008084491683471, -0.011984599277133,
	      	      -0.007210626507497,  0.008081198471645, -0.999941316450383, -0.054039847297480,
	      	       0.999973864590328,  0.000485948581039, -0.007206933692422, -0.292196864868591,
	      	       0                ,  0                ,  0                ,  1.000000000000000;
      else if (sequence == 3)
              // 03: 2011_09_26_drive_0067
            calibration << 0.000234773698147, -0.999944154543764, -0.010563477811052, -0.002796816941295,
                           0.010449407416593,  0.010565353641379, -0.999889574117649, -0.075108791382965,
                           0.999945388562002,  0.000124365378387,  0.010451302995669, -0.272132796405873,
                           0                ,  0                ,  0                ,  1.000000000000000;
      else if (sequence == -1)
            calibration << 1, 0, 0, 0,
                            0, 1, 0, 0,
                            0, 0, 1, 0,
                            0, 0, 0, 1;
      else
           calibration <<  -0.001857739385241, -0.999965951350955, -0.008039975204516, -0.004784029760483,
                           -0.006481465826011,  0.008051860151134, -0.999946608177406, -0.073374294642306,
                            0.999977309828677, -0.001805528627661, -0.006496203536139, -0.333996806443304,
                            0                ,  0                ,  0                ,  1.000000000000000;
    }

    semantic_bki::PCLPointCloud::Ptr kitti2pclflow(const std::string &fn, 
                                                  const std::string &fn_flow,
                                                  const std::string &fn_label){
      FILE* fp_label = std::fopen(fn_label.c_str(), "r");
      //label file
      if (!fp_label)
        std::perror("File opening failed: label");
      
      std::fseek(fp_label, 0L, SEEK_END);
      std::rewind(fp_label);

      //flow file
      FILE* fp_flow = std::fopen(fn_flow.c_str(), "r");
      if (!fp_flow)
        std::perror("File opening failed: flow");
      
      std::fseek(fp_flow, 0L, SEEK_END);
      std::rewind(fp_flow);

      //xyz file
      FILE* fp = std::fopen(fn.c_str(), "r");
      if (!fp)
        std::perror("File opening failed: velodyne");
      
      std::fseek(fp, 0L, SEEK_END);
      size_t sz = std::ftell(fp);
      std::rewind(fp);

      int n_hits = sz / (sizeof(float) * 4);

      semantic_bki::PCLPointCloud::Ptr pc(new semantic_bki::PCLPointCloud);
      for (size_t i = 0; i < n_hits; i++){
        semantic_bki::PCLPointType point;
        float intensity;
        if (fread(&point.x, sizeof(float), 1, fp) == 0) break;
        if (fread(&point.y, sizeof(float), 1, fp) == 0) break;
        if (fread(&point.z, sizeof(float), 1, fp) == 0) break;
        if (fread(&intensity, sizeof(float), 1, fp) == 0) break;
        if (fread(&point.vx, sizeof(float), 1, fp_flow) == 0) break;
        if (fread(&point.vy, sizeof(float), 1, fp_flow) == 0) break;
        if (fread(&point.vz, sizeof(float), 1, fp_flow) == 0) break;
        if (fread(&point.label, sizeof(float), 1, fp_label) == 0) break;
        pc->push_back(point);

      }

      std::fclose(fp);
      std::fclose(fp_flow);
      std::fclose(fp_label);
      return pc;    
    }

    pcl::PointCloud<pcl::PointXYZL>::Ptr kitti2pcl(const std::string &fn, 
                                                  const std::string &fn_label) {
      FILE* fp_label = std::fopen(fn_label.c_str(), "r");
      if (!fp_label) {
        std::perror("File opening failed");
      }
      std::fseek(fp_label, 0L, SEEK_END);
      std::rewind(fp_label);
      FILE* fp = std::fopen(fn.c_str(), "r");
      if (!fp) {
        std::perror("File opening failed");
      }
      std::fseek(fp, 0L, SEEK_END);
      size_t sz = std::ftell(fp);
      std::rewind(fp);
      int n_hits = sz / (sizeof(float) * 4);
      pcl::PointCloud<pcl::PointXYZL>::Ptr pc(new pcl::PointCloud<pcl::PointXYZL>);
      for (int i = 0; i < n_hits; i++) {
        pcl::PointXYZL point;
        float intensity;
        if (fread(&point.x, sizeof(float), 1, fp) == 0) break;
        if (fread(&point.y, sizeof(float), 1, fp) == 0) break;
        if (fread(&point.z, sizeof(float), 1, fp) == 0) break;
        if (fread(&intensity, sizeof(float), 1, fp) == 0) break;
        if (fread(&point.label, sizeof(float), 1, fp_label) == 0) break;
        pc->push_back(point);
      }
      std::fclose(fp);
      std::fclose(fp_label);
      return pc;
    }

    semantic_bki::PCLPointCloud::Ptr get_one_scan(const std::string &input_data_dir,
						  const std::string &input_label_dir,
						  const int scan_id,
						  semantic_bki::point3f &origin){
        char scan_id_c[256];
        sprintf(scan_id_c, "%06d", scan_id);
        std::string scan_name = input_data_dir + std::string(scan_id_c) + ".bin";
        std::string label_name = input_label_dir + std::string(scan_id_c) + ".label";
        std::string flow_name = input_label_dir + std::string(scan_id_c) + ".bin";
	      semantic_bki::PCLPointCloud::Ptr cloud = kitti2pclflow(scan_name, flow_name, label_name);

	      Eigen::Matrix4d transform = lidar_poses_[scan_id];
	      Eigen::Matrix4d new_transform = init_trans_to_ground_ * transform * calibration;
        pcl::transformPointCloud (*cloud, *cloud, new_transform);
        origin.x() = new_transform(0, 3);
        origin.y() = new_transform(1, 3);
        origin.z() = new_transform(2, 3);

	      return cloud;
    }
};
