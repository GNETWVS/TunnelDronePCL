#ifndef STITCHED_CLOUD_H
#define STITCHED_CLOUD_H

#include <pcl/common/common.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_representation.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/features/pfh.h>
#include <pcl/surface/mls.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

/*          Ways to improve registration        */
// - outlier removal: number of neighbours, standard deviation
// - downsampling: leaf size


// Store any known transformations for point clouds
struct TransformData
{
    double dx = 0;
    double dy = 0;
    double dz = 0;
    double rotx = 0;
    double roty = 0;
    double rotz = 0;
    double confidence = 0;  // 0-1; used to determine number of registration iterations
};

// The resultant cloud after multiple point clouds have been registered
class StitchedCloud
{
public:
    StitchedCloud(PointCloudT::Ptr point_cloud);
    void addCloud(PointCloudT::Ptr new_cloud, const TransformData& transformation);

    PointCloudT::Ptr stitched_cloud;    // Avoids using boost shared pointers
private:
    void registerWithICP(PointCloudT::Ptr cloud, const int iters);
    void registerWithSAC(PointCloudT::Ptr cloud, const int iters);
};

// Helper functions
void removeOutliers(PointCloudT::Ptr cloud, const int num_neighbours, const int stddev);
void downSample(PointCloudT::Ptr cloud, const int leaf_size);
void transform(PointCloudT::Ptr cloud, const TransformData& t);
void filterRangeZ(PointCloudT::Ptr cloud, const double minZ, const double maxZ);

#endif
