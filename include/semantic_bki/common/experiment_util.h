#include "bkioctomap.h"
#include "markerarray_pub.h"


static void load_pcd(std::string filename, semantic_bki::point3f &origin,
                 semantic_bki::PCLPointCloud &cloud, bool rotate = false) {
    pcl::PCLPointCloud2 cloud2;
    Eigen::Vector4f _origin;
    if (rotate){
        semantic_bki::PCLPointCloud rot_cloud;
        Eigen::Vector3f _zero_origin = Eigen::Matrix< float, 3, 1 >::Zero();
    }
    Eigen::Quaternionf orientation;
    pcl::io::loadPCDFile(filename, cloud2, _origin, orientation);
    pcl::fromPCLPointCloud2(cloud2, cloud);
    if (rotate){
        _zero_origin[0] = _origin[0];
        _zero_origin[1] = _origin[1];
        _zero_origin[2] = _origin[2];
        pcl::transformPointCloud(rot_cloud, cloud, _zero_origin, orientation);
    }
    
    origin.x() = _origin[0];
    origin.y() = _origin[1];
    origin.z() = _origin[2];
}