#include <algorithm>

#include "bkioctomap.h"
#include "bki.h"

using std::vector;

// #define DEBUG true;

#ifdef DEBUG

#include <iostream>

#define Debug_Msg(msg) {\
std::cout << "Debug: " << msg << std::endl; }
#endif

namespace semantic_bki {

    SemanticBKIOctoMap::SemanticBKIOctoMap() : SemanticBKIOctoMap(0.1f, // resolution
                                        4, // block_depth
                                        3,  // num_class
                                        1.0, // sf2
                                        1.0, // ell
                                        1.0f, // prior
                                        0.2, //flow_sf2
                                        0.2, //flow_ell
                                        1.0f, // var_thresh
                                        0.3f, // free_thresh
                                        0.7f, // occupied_thresh
                                        false
                                    ) { }
    SemanticBKIOctoMap::SemanticBKIOctoMap(const MapParams *params) : SemanticBKIOctoMap(params->resolution, // resolution
                                        params->block_depth,
                                        params->num_classes,
                                        params->sf2, // sf2
                                        params->ell, // ell
                                        params->prior, // prior
                                        params->flow_sf2,
                                        params->flow_ell,
                                        params->var_thresh, // var_thresh
                                        params->free_thresh,
                                        params->occupied_thresh, // occupied_thresh
                                        params->spatiotemporal
                                    ) { }

    SemanticBKIOctoMap::SemanticBKIOctoMap(const float resolution,
                        const unsigned short block_depth,
                        const int num_class,
                        const float sf2,
                        const float ell,
                        const float prior,
                        const float flow_sf2,
                        const float flow_ell,
                        const float var_thresh,
                        const float free_thresh,
                        const float occupied_thresh,
                        const bool spatiotemporal_in)
            : resolution(resolution), block_depth(block_depth),
              block_size((float) pow(2, block_depth - 1) * resolution),
              spatiotemporal(spatiotemporal_in) {
        Block::resolution = resolution;
        Block::size = this->block_size;
        Block::key_loc_map = init_key_loc_map(resolution, block_depth);
        Block::index_map = init_index_map(Block::key_loc_map, block_depth);
        
        // Bug fixed
        Block::cell_num = static_cast<unsigned short>(round(Block::size / Block::resolution));
        std::cout << "block::resolution: " << Block::resolution << std::endl;
        std::cout << "block::size: " << Block::size << std::endl;
        std::cout << "block::cell_num: " << Block::cell_num << std::endl;
        
        SemanticOcTree::max_depth = block_depth;

        SemanticOcTreeNode::num_class = num_class;
        SemanticOcTreeNode::kp.sf2 = sf2;
        SemanticOcTreeNode::kp.ell = ell;
        SemanticOcTreeNode::prior = prior;
        SemanticOcTreeNode::kp.flow_sf2 = flow_sf2;
        SemanticOcTreeNode::kp.flow_ell = flow_ell;
        SemanticOcTreeNode::var_thresh = var_thresh;
        SemanticOcTreeNode::free_thresh = free_thresh;
        SemanticOcTreeNode::occupied_thresh = occupied_thresh;
    }

    SemanticBKIOctoMap::~SemanticBKIOctoMap() {
        for (auto it = block_arr.begin(); it != block_arr.end(); ++it) {
            if (it->second != nullptr) {
                delete it->second;
            }
        }
    }

    void SemanticBKIOctoMap::set_resolution(float resolution) {
        this->resolution = resolution;
        Block::resolution = resolution;
        this->block_size = (float) pow(2, block_depth - 1) * resolution;
        Block::size = this->block_size;
        Block::key_loc_map = init_key_loc_map(resolution, block_depth);
    }

    void SemanticBKIOctoMap::set_block_depth(unsigned short max_depth) {
        this->block_depth = max_depth;
        SemanticOcTree::max_depth = max_depth;
        this->block_size = (float) pow(2, block_depth - 1) * resolution;
        Block::size = this->block_size;
        Block::key_loc_map = init_key_loc_map(resolution, block_depth);
    }

    void SemanticBKIOctoMap::insert_pointcloud_csm(const PCLPointCloud &cloud, const point3f &origin, 
                                                 const PCParams *train_params, const ScanStep create_id) {

#ifdef DEBUG
        Debug_Msg("Insert pointcloud: " << "cloud size: " << cloud.size() << " origin: " << origin);
#endif

        ////////// Preparation //////////////////////////
        /////////////////////////////////////////////////
        GPPointCloud xy;
        get_training_data(cloud, origin, train_params, xy);
#ifdef DEBUG
        Debug_Msg("Training data size: " << xy.size());
#endif
        // If pointcloud after max_range filtering is empty
        //  no need to do anything
        if (xy.size() == 0) {
            return;
        }

        point3f lim_min, lim_max;
        bbox(xy, lim_min, lim_max); //6D point here

        vector<BlockHashKey> blocks;
        get_blocks_in_bbox(lim_min, lim_max, blocks); //gives keys to blocks

        for (auto it = xy.cbegin(); it != xy.cend(); ++it) {
            float p[] = {it->first.x(), it->first.y(), it->first.z()};
            rtree.Insert(p, p, const_cast<GPPointType *>(&*it));
        }
        /////////////////////////////////////////////////

        ////////// Training /////////////////////////////
        /////////////////////////////////////////////////
        vector<BlockHashKey> test_blocks;
        std::unordered_map<BlockHashKey, SemanticBKI3f *> bgk_arr;
#ifdef OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
        for (int i = 0; i < blocks.size(); ++i) {
            BlockHashKey key = blocks[i];
            ExtendedBlock eblock = get_extended_block(key);
            if (has_gp_points_in_bbox(eblock))
            //if there are points present in the extended block of this block:
#ifdef OPENMP
#pragma omp critical
#endif
            {
                test_blocks.push_back(key);
                //store this block for future inference
            };

            GPPointCloud block_xy; // a 6D point cloud with an attached label
            get_gp_points_in_bbox(key, block_xy);
            //get the point cloud that lies exclusively in this block
            if (block_xy.size() < 1)
                continue;

            vector<float> block_x, block_v, block_y;
            for (auto it = block_xy.cbegin(); it != block_xy.cend(); ++it) {
                block_x.push_back(it->first.x());
                block_x.push_back(it->first.y());
                block_x.push_back(it->first.z());
                block_v.push_back(it->first.flow_norm());
                //block_v.push_back(it->first.vy());
                //block_v.push_back(it->first.vz());
                block_y.push_back(it->second);
                //process the points contiguously to associate with a label
            //Shwarya TODO: Store the velocities somehow in bgk_arr as well?
            //Because these are the flows associated with those points anyway
            
            //std::cout << search(it->first.x(), it->first.y(), it->first.z()) << std::endl;
            }

            SemanticBKI3f *bgk = new SemanticBKI3f(SemanticOcTreeNode::num_class, spatiotemporal, SemanticOcTreeNode::kp);
            bgk->train(block_x, block_y);
            //stores the training points into an Inference instance.
            bgk->store_flow(block_v);
#ifdef OPENMP
#pragma omp critical
#endif
            {
                bgk_arr.emplace(key, bgk);
            };
        }
#ifdef DEBUG
        Debug_Msg("Training done");
        Debug_Msg("Prediction: block number: " << test_blocks.size());
#endif
        /////////////////////////////////////////////////

        ////////// Prediction ///////////////////////////
        /////////////////////////////////////////////////
#ifdef OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
        for (int i = 0; i < test_blocks.size(); ++i) {
            BlockHashKey key = test_blocks[i];
#ifdef OPENMP
#pragma omp critical
#endif
            {
                if (block_arr.find(key) == block_arr.end())
                    block_arr.emplace(key, new Block(hash_key_to_block(key), create_id));
                    //couldn't find a block corresponding to the HashKey
                    //initialize new Block
                    //create_id can be useful in the future for time difference
            };
            Block *block = block_arr[key];
            vector<float> xs;
            //obtain all the octree points that exist within the block (unpruned)
            for (auto leaf_it = block->begin_leaf(); leaf_it != block->end_leaf(); ++leaf_it) {
                point3f p = block->get_loc(leaf_it);
                xs.push_back(p.x());
                xs.push_back(p.y());
                xs.push_back(p.z());
            }
            //std::cout << "xs size: "<<xs.size() << std::endl;

	        // For counting sensor model
            //get training points in the local block
            auto bgk = bgk_arr.find(key);
            if (bgk == bgk_arr.end())
              continue;

            vector<vector<float>> ybars, vbars;
            bgk->second->predict_csm(xs, ybars, vbars);

            int j = 0;
            for (auto leaf_it = block->begin_leaf(); leaf_it != block->end_leaf(); ++leaf_it, ++j) {
                SemanticOcTreeNode &node = leaf_it.get_node();

                // Only need to update if kernel density total kernel density est > 0
                node.update(ybars[j], vbars[j], spatiotemporal);
                //Note: Shwarya : only potential place to update velocities or 
                // "store" any type of parameters.
            }

        }
#ifdef DEBUG
        Debug_Msg("Prediction done");
#endif

        ////////// Cleaning /////////////////////////////
        /////////////////////////////////////////////////
        for (auto it = bgk_arr.begin(); it != bgk_arr.end(); ++it)
            delete it->second;

        rtree.RemoveAll();
    }



    void SemanticBKIOctoMap::insert_pointcloud(const PCLPointCloud &cloud, const point3f &origin, 
                                                const PCParams *train_params, const ScanStep create_id) {

#ifdef DEBUG
        Debug_Msg("Insert pointcloud: " << "cloud size: " << cloud.size() << " origin: " << origin);
#endif

        ////////// Preparation //////////////////////////
        /////////////////////////////////////////////////
        GPPointCloud xy;
        get_training_data(cloud, origin, train_params, xy);
#ifdef DEBUG
        Debug_Msg("Training data size: " << xy.size());
#endif
        // If pointcloud after max_range filtering is empty
        //  no need to do anything
        if (xy.size() == 0) {
            return;
        }

        point3f lim_min, lim_max;
        bbox(xy, lim_min, lim_max);

        vector<BlockHashKey> blocks;
        get_blocks_in_bbox(lim_min, lim_max, blocks);

        for (auto it = xy.cbegin(); it != xy.cend(); ++it) {
            float p[] = {it->first.x(), it->first.y(), it->first.z()};
            rtree.Insert(p, p, const_cast<GPPointType *>(&*it));
        }
        /////////////////////////////////////////////////

        ////////// Training /////////////////////////////
        /////////////////////////////////////////////////
        vector<BlockHashKey> test_blocks;
        std::unordered_map<BlockHashKey, SemanticBKI3f *> bgk_arr;
#ifdef OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
        for (int i = 0; i < blocks.size(); ++i) {
            BlockHashKey key = blocks[i];
            ExtendedBlock eblock = get_extended_block(key);
            if (has_gp_points_in_bbox(eblock))
#ifdef OPENMP
#pragma omp critical
#endif
            {
                test_blocks.push_back(key);
            };

            GPPointCloud block_xy;
            get_gp_points_in_bbox(key, block_xy);
            if (block_xy.size() < 1)
                continue;

            vector<float> block_x, block_v, block_y;
            for (auto it = block_xy.cbegin(); it != block_xy.cend(); ++it) {
                block_x.push_back(it->first.x());
                block_x.push_back(it->first.y());
                block_x.push_back(it->first.z());
                block_v.push_back(it->first.flow_norm());
                block_y.push_back(it->second); //label
            
            
            //std::cout << search(it->first.x(), it->first.y(), it->first.z()) << std::endl;
            }

            SemanticBKI3f *bgk = new SemanticBKI3f(SemanticOcTreeNode::num_class, spatiotemporal, SemanticOcTreeNode::kp);
            bgk->train(block_x, block_y);
            bgk->store_flow(block_v);
#ifdef OPENMP
#pragma omp critical
#endif
            {
                bgk_arr.emplace(key, bgk);
            };
        }
#ifdef DEBUG
        Debug_Msg("Training done");
        Debug_Msg("Prediction: block number: " << test_blocks.size());
#endif
        /////////////////////////////////////////////////

        ////////// Prediction ///////////////////////////
        /////////////////////////////////////////////////
#ifdef OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
        for (int i = 0; i < test_blocks.size(); ++i) {
            BlockHashKey key = test_blocks[i];
#ifdef OPENMP
#pragma omp critical
#endif
            {
                if (block_arr.find(key) == block_arr.end())
                    block_arr.emplace(key, new Block(hash_key_to_block(key), create_id));
            };
            Block *block = block_arr[key];
            vector<float> xs;
            for (auto leaf_it = block->begin_leaf(); leaf_it != block->end_leaf(); ++leaf_it) {
                point3f p = block->get_loc(leaf_it);
                xs.push_back(p.x());
                xs.push_back(p.y());
                xs.push_back(p.z());
            }
            //std::cout << "xs size: "<<xs.size() << std::endl;

            ExtendedBlock eblock = block->get_extended_block();
            for (auto block_it = eblock.cbegin(); block_it != eblock.cend(); ++block_it) {
                auto bgk = bgk_arr.find(*block_it);
                if (bgk == bgk_arr.end())
                    continue;

               	vector<vector<float>> ybars, vbars;
		            bgk->second->predict(xs, ybars, vbars);

                int j = 0;
                for (auto leaf_it = block->begin_leaf(); leaf_it != block->end_leaf(); ++leaf_it, ++j) {
                    SemanticOcTreeNode &node = leaf_it.get_node();
                    // Only need to update if kernel density total kernel density est > 0
                    //if (kbar[j] > 0.0)
                    node.update(ybars[j], vbars[j], spatiotemporal);
                    // node.update(ybars[j], vbars[j], create_id - block->created_at);
                    // block->created_at = create_id;
                }
            }
        }
#ifdef DEBUG
        Debug_Msg("Prediction done");
#endif

        ////////// Cleaning /////////////////////////////
        /////////////////////////////////////////////////
        for (auto it = bgk_arr.begin(); it != bgk_arr.end(); ++it)
            delete it->second;

        rtree.RemoveAll();
    }

    // void SemanticBKIOctoMap::get_bbox(point3f &lim_min, point3f &lim_max) const {
    //     lim_min = point3f(0, 0, 0);
    //     lim_max = point3f(0, 0, 0);

    //     GPPointCloud centers;
    //     for (auto it = block_arr.cbegin(); it != block_arr.cend(); ++it) {
    //         //block_arr iterator is going to return a pair.
    //         centers.emplace_back(it->second->get_center(), 1);
    //     }
    //     if (centers.size() > 0) {
    //         bbox(centers, lim_min, lim_max);
    //         lim_min -= point3f(block_size, block_size, block_size) * 0.5;
    //         lim_max += point3f(block_size, block_size, block_size) * 0.5;
    //     }
    // }

    void SemanticBKIOctoMap::get_training_data(const PCLPointCloud &cloud, const point3f &origin, 
                                                const PCParams *train_params, GPPointCloud &xy) const {
        PCLPointCloud sampled_hits;
        downsample(cloud, sampled_hits, train_params->ds_resolution);

        PCLPointCloud frees;
        frees.height = 1;
        frees.width = 0;
        xy.clear();
        for (auto it = sampled_hits.begin(); it != sampled_hits.end(); ++it) {
            flow3f p(it->x, it->y, it->z, it->vx, it->vy, it->vz);
            if (train_params->max_range > 0) {
                double l = (p.point() - origin).norm();
                if (l > train_params->max_range)
                    continue;
            }
            
            xy.emplace_back(p, it->label); //don't recommend modifying
            //xy because it is fed into the RTree for spatial partitioning

            //frees need not have a velocity associated with them

            PointCloud frees_n;
            beam_sample(p, origin, frees_n, train_params->free_resolution);

            PCLPointType p_origin = PCLPointType();
            p_origin.x = origin.x();
            p_origin.y = origin.y();
            p_origin.z = origin.z();
            p_origin.label = 0;
            frees.push_back(p_origin);
            
            for (auto p = frees_n.begin(); p != frees_n.end(); ++p) {
                PCLPointType p_free = PCLPointType();
                p_free.x = p->x();
                p_free.y = p->y();
                p_free.z = p->z();
                p_free.label = 0;
                frees.push_back(p_free);
                frees.width++;
            }
        }

        PCLPointCloud sampled_frees;    
        downsample(frees, sampled_frees, train_params->ds_resolution);

        //frees are also passed into xy

        for (auto it = sampled_frees.begin(); it != sampled_frees.end(); ++it) {
            xy.emplace_back(flow3f(it->x, it->y, it->z, 0.0, 0.0, 0.0), 0.0f);
        }
    }

    void SemanticBKIOctoMap::downsample(const PCLPointCloud &in, PCLPointCloud &out, const float ds_resolution) const {
        if (ds_resolution < 0) {
            out = in;
            return;
        }

        PCLPointCloud::Ptr pcl_in(new PCLPointCloud(in));

        pcl::VoxelGrid<PCLPointType> sor;
        sor.setInputCloud(pcl_in);
        sor.setLeafSize(ds_resolution, ds_resolution, ds_resolution);
        sor.filter(out);
    }

    void SemanticBKIOctoMap::beam_sample(const flow3f &hit, const point3f &origin, PointCloud &frees,
                                const float free_resolution) const {
        frees.clear();

        float x0 = origin.x();
        float y0 = origin.y();
        float z0 = origin.z();

        float x = hit.x();
        float y = hit.y();
        float z = hit.z();

        float l = (float) sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0) + (z - z0) * (z - z0));

        float nx = (x - x0) / l;
        float ny = (y - y0) / l;
        float nz = (z - z0) / l;

        float d = free_resolution;
        while (d < l) {
            frees.emplace_back(x0 + nx * d, y0 + ny * d, z0 + nz * d);
            d += free_resolution;
        }
        if (l > free_resolution)
            frees.emplace_back(x0 + nx * (l - free_resolution), y0 + ny * (l - free_resolution), z0 + nz * (l - free_resolution));
    }

    /*
     * Compute bounding box of pointcloud
     * Precondition: cloud non-empty
     */
    void SemanticBKIOctoMap::bbox(const GPPointCloud &cloud, point3f &lim_min, point3f &lim_max) const {
        assert(cloud.size() > 0);
        vector<float> x, y, z;
        for (auto it = cloud.cbegin(); it != cloud.cend(); ++it) {
            x.push_back(it->first.x());
            y.push_back(it->first.y());
            z.push_back(it->first.z());
        }

        auto xlim = std::minmax_element(x.cbegin(), x.cend());
        auto ylim = std::minmax_element(y.cbegin(), y.cend());
        auto zlim = std::minmax_element(z.cbegin(), z.cend());

        lim_min.x() = *xlim.first;
        lim_min.y() = *ylim.first;
        lim_min.z() = *zlim.first;

        lim_max.x() = *xlim.second;
        lim_max.y() = *ylim.second;
        lim_max.z() = *zlim.second;
    }

    void SemanticBKIOctoMap::get_blocks_in_bbox(const point3f &lim_min, const point3f &lim_max,
                                       vector<BlockHashKey> &blocks) const {
        for (float x = lim_min.x() - block_size; x <= lim_max.x() + 2 * block_size; x += block_size) {
            for (float y = lim_min.y() - block_size; y <= lim_max.y() + 2 * block_size; y += block_size) {
                for (float z = lim_min.z() - block_size; z <= lim_max.z() + 2 * block_size; z += block_size) {
                    blocks.push_back(block_to_hash_key(x, y, z));
                }
            }
        }
    }

    int SemanticBKIOctoMap::get_gp_points_in_bbox(const BlockHashKey &key,
                                         GPPointCloud &out) {
        point3f half_size(block_size / 2.0f, block_size / 2.0f, block_size / 2.0);
        point3f lim_min = hash_key_to_block(key) - half_size;
        point3f lim_max = hash_key_to_block(key) + half_size;
        return get_gp_points_in_bbox(lim_min, lim_max, out);
    }

    int SemanticBKIOctoMap::has_gp_points_in_bbox(const BlockHashKey &key) {
        point3f half_size(block_size / 2.0f, block_size / 2.0f, block_size / 2.0);
        point3f lim_min = hash_key_to_block(key) - half_size;
        point3f lim_max = hash_key_to_block(key) + half_size;
        return has_gp_points_in_bbox(lim_min, lim_max);
    }

    int SemanticBKIOctoMap::get_gp_points_in_bbox(const point3f &lim_min, const point3f &lim_max,
                                         GPPointCloud &out) {
        float a_min[] = {lim_min.x(), lim_min.y(), lim_min.z()};
        float a_max[] = {lim_max.x(), lim_max.y(), lim_max.z()};
        return rtree.Search(a_min, a_max, SemanticBKIOctoMap::search_callback, static_cast<void *>(&out));
    }

    int SemanticBKIOctoMap::has_gp_points_in_bbox(const point3f &lim_min,
                                         const point3f &lim_max) {
        float a_min[] = {lim_min.x(), lim_min.y(), lim_min.z()};
        float a_max[] = {lim_max.x(), lim_max.y(), lim_max.z()};
        return rtree.Search(a_min, a_max, SemanticBKIOctoMap::count_callback, NULL);
    }

    bool SemanticBKIOctoMap::count_callback(GPPointType *p, void *arg) {
        return false;
    }

    bool SemanticBKIOctoMap::search_callback(GPPointType *p, void *arg) {
        GPPointCloud *out = static_cast<GPPointCloud *>(arg);
        out->push_back(*p);
        return true;
    }


    int SemanticBKIOctoMap::has_gp_points_in_bbox(const ExtendedBlock &block) {
        for (auto it = block.cbegin(); it != block.cend(); ++it) {
            if (has_gp_points_in_bbox(*it) > 0)
                return 1;
        }
        return 0;
    }

    int SemanticBKIOctoMap::get_gp_points_in_bbox(const ExtendedBlock &block,
                                         GPPointCloud &out) {
        int n = 0;
        for (auto it = block.cbegin(); it != block.cend(); ++it) {
            n += get_gp_points_in_bbox(*it, out);
        }
        return n;
    }

    Block *SemanticBKIOctoMap::search(BlockHashKey key) const {
        auto block = block_arr.find(key);
        if (block == block_arr.end()) {
            return nullptr;
        } else {
            return block->second;
        }
    }

    SemanticOcTreeNode SemanticBKIOctoMap::search(point3f p) const {
        Block *block = search(block_to_hash_key(p));
        if (block == nullptr) {
          return SemanticOcTreeNode();
        } else {
          return SemanticOcTreeNode(block->search(p));
        }
    }

    SemanticOcTreeNode SemanticBKIOctoMap::search(float x, float y, float z) const {
        return search(point3f(x, y, z));
    }

    void SemanticBKIOctoMap::sync_block(ScanStep scans_done){
        for (auto it = block_arr.begin(); it != block_arr.end(); ++it) {
            Block *curr_block = it->second;
            for (auto leaf_it = curr_block->begin_leaf(); leaf_it != curr_block->end_leaf(); ++leaf_it) {
                SemanticOcTreeNode &node = leaf_it.get_node();
                //update the OctreeNode with posterior predictive distribution
                node.pred_post_update(scans_done - curr_block->created_at);
            }
        }
    }

}
