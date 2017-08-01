#ifndef STITCHED_CLOUD_H
#define STITCHED_CLOUD_H

#include <chrono>

#include <boost/progress.hpp>

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
#include <pcl/features/normal_3d_omp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transforms.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/surface/mls.h>
#include <pcl/registration/ndt.h>
#include <pcl/features/fpfh_omp.h>

typedef pcl::PointXYZ PointT;
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

    TransformData operator+(const TransformData& other)
    {
      TransformData t;
      t.dx = this->dx + other.dx;
      t.dy = this->dy + other.dy;
      t.dz = this->dz + other.dz;
      t.rotx = this->rotx + other.rotx;
      t.roty = this->roty + other.roty;
      t.rotz = this->rotz + other.rotz;
      t.confidence = this->confidence + other.confidence;
      return t;
    }
    TransformData operator-(const TransformData& other)
    {
      TransformData t;
      t.dx = this->dx - other.dx;
      t.dy = this->dy - other.dy;
      t.dz = this->dz - other.dz;
      t.rotx = this->rotx - other.rotx;
      t.roty = this->roty - other.roty;
      t.rotz = this->rotz - other.rotz;
      t.confidence = this->confidence - other.confidence;
      return t;
    }
};

struct TimeBreakdown
{
    // Store the time in seconds to complete various operations
    std::chrono::duration<double, std::ratio<1, 1000> > read_write_time;
    std::chrono::duration<double, std::ratio<1, 1000> > downsample_time;
    std::chrono::duration<double, std::ratio<1, 1000> > sor_time;
    std::chrono::duration<double, std::ratio<1, 1000> > transform_time;
    std::chrono::duration<double, std::ratio<1, 1000> > passthrough_time;
    std::chrono::duration<double, std::ratio<1, 1000> > icp_time;
    std::chrono::duration<double, std::ratio<1, 1000> > sac_time;
    std::chrono::duration<double, std::ratio<1, 1000> > smooth_time;
    std::chrono::duration<double, std::ratio<1, 1000> > total_time;

    void print()
    {
        read_write_time = total_time - downsample_time - sor_time - transform_time - passthrough_time
                                     - icp_time - sac_time - smooth_time;
        const int w = 4;
        auto orig_prec = std::cout.precision();
        std::cout << std::setprecision(2)
                  << "Processing Time Breakdown\n"
                  << "_____________________________\n"
                  << std::setw(23) << "Read/Write\t" << std::setw(w) << round(100*read_write_time.count()/total_time.count()) << "%\n"
                  << std::setw(23) << "Downsampling\t" << std::setw(w) << round(100*downsample_time.count()/total_time.count()) << "%\n"
                  << std::setw(23) << "Outlier removal\t" << std::setw(w) << round(100*sor_time.count()/total_time.count()) << "%\n"
                  << std::setw(23) << "Transformation\t" << std::setw(w) << round(100*transform_time.count()/total_time.count()) << "%\n"
                  << std::setw(23) << "Passthrough filter\t" << std::setw(w) << round(100*passthrough_time.count()/total_time.count()) << "% \n"
                  << std::setw(23) << "ICP\t" << std::setw(w) << round(100*icp_time.count()/total_time.count()) << "%\n"
                  << std::setw(23) << "SAC IA\t" << std::setw(w) << round(100*sac_time.count()/total_time.count()) << "%\n"
                  << std::setw(23) << "Surface reconstruction\t" << std::setw(w) << round(100*smooth_time.count()/total_time.count()) << "%\n"
                  << "=============================\n"
                  << std::setw(23) << "Total time" << std::setprecision(orig_prec) << std::setw(w+10) << total_time.count() << "s\n"
                  << "_____________________________\n" << std::endl;
    }
};

// The resultant cloud after multiple point clouds have been registered
class StitchedCloud
{
public:
    StitchedCloud(PointCloudT::Ptr point_cloud);
    void addCloud(PointCloudT::Ptr new_cloud, const TransformData& transformation);

    PointCloudT::Ptr stitched_cloud;    // Avoids using boost shared pointers

    TimeBreakdown timeBreakdown;
private:
    // Helper functions
    void registerWithICP(PointCloudT::Ptr cloud, const int iters);
    void registerWithSAC(PointCloudT::Ptr cloud, const int iters);
    void removeOutliers(PointCloudT::Ptr cloud, const int num_neighbours, const int stddev);
    void downSample(PointCloudT::Ptr cloud, const int leaf_size);
    void transform(PointCloudT::Ptr cloud, const TransformData& t);
    void filterRangeZ(PointCloudT::Ptr cloud, const double minZ, const double maxZ);
    void reconstructSurface(pcl::PointCloud<pcl::PointNormal>::Ptr mls_points, const PointCloudT::Ptr cloud, const double radius);
    void smoothSurface(PointCloudT::Ptr cloud, const double radius);
};


#endif
