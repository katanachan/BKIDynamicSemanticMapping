#ifndef FLOW_POINT_TYPES_H
#define FLOW_POINT_TYPES_H
#define PCL_NO_PRECOMPILE
//#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/pcl_macros.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

namespace semantic_bki{

   //New Point Cloud that is annotated with the flow in each dimension
    // Computed with FlowNet3d as of 8/02/20

    struct XYZFlowL
    {
        PCL_ADD_POINT4D;
        float vx;
        float vy;
        float vz;
        int label;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    } EIGEN_ALIGN16;

    struct XYZLGT{
        PCL_ADD_POINT4D;
        int label;
        int gt;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    } EIGEN_ALIGN16;

    struct XYZFlowLGT{
        PCL_ADD_POINT4D;
        float vx;
        float vy;
        float vz;
        int label;
        int gt;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    } EIGEN_ALIGN16;

}

POINT_CLOUD_REGISTER_POINT_STRUCT (semantic_bki::XYZFlowL,           // here we assume a XYZ + "test" (as fields)
                                (float, x, x)
                                (float, y, y)
                                (float, z, z)
                                (float, vx, vx)
				                (float, vy, vy)
				                (float, vz, vz)
				                (int, label, label)
				                ) 
    //declared inside namespace, defined outside namespace.
POINT_CLOUD_REGISTER_POINT_STRUCT (semantic_bki::XYZFlowLGT,     
                                (float, x, x)
                                (float, y, y)
                                (float, z, z)
                                (float, vx, vx)
				                (float, vy, vy)
				                (float, vz, vz)
				                (int, label, label)
                                (int, gt, gt)
				                ) 

POINT_CLOUD_REGISTER_POINT_STRUCT (semantic_bki::XYZLGT,     
                                (float, x, x)
                                (float, y, y)
                                (float, z, z)
				                (int, label, label)
                                (int, gt, gt)
				                ) 

namespace semantic_bki{
    typedef XYZFlowL PCLPointType;


    typedef pcl::PointCloud<XYZFlowL> PCLPointCloud;

    typedef pcl::PointCloud<XYZLGT> SKittiTrainPC;
    typedef XYZLGT SKittiPointType; 
    //typedef pcl::PointCloud<XYZFlowGT>

    // // PCL PointCloud types as input
    // typedef pcl::PointXYZL PCLPointType;
    // typedef pcl::PointCloud<PCLPointType> PCLPointCloud;
}
#endif
