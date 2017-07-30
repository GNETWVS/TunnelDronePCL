#include <stitched_cloud.h>

StitchedCloud::StitchedCloud(PointCloudT::Ptr point_cloud)
{
    stitched_cloud = point_cloud;
    downSample(stitched_cloud, 500);
    removeOutliers(stitched_cloud, 500, 1);
    filterRangeZ(stitched_cloud, 0, 10000);
}

void StitchedCloud::addCloud(PointCloudT::Ptr new_cloud, const TransformData& transformation)
{
    // Uses given transformation data to improve the speed of registration

    /*          Pre-Processing          */
    downSample(new_cloud, 500);
    removeOutliers(new_cloud, 500, 2);
    transform(new_cloud, transformation);
    filterRangeZ(new_cloud, transformation.dz, 10000+transformation.dz);
    /*          Registration            */
    // registerWithSAC(new_cloud, 2);
    registerWithICP(new_cloud, 100);
    *stitched_cloud += *new_cloud;
    /*          Post-Processing         */
    removeOutliers(stitched_cloud, 50, 2);
}

void StitchedCloud::registerWithICP(PointCloudT::Ptr cloud, const int iters)
{
    // Transforms 'cloud' such that it more closely aligns with the stitched cloud
    // Accurate results but slow run time
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setMaximumIterations(iters);
    icp.setInputSource(cloud);
    icp.setInputTarget(stitched_cloud);
    PointCloudT::Ptr registered_cloud (new PointCloudT());
    icp.align(*cloud);

    // Revert the translation in the z axis
    TransformData t;
    t.dz = -icp.getFinalTransformation()(2,3);
    transform(cloud, t);
}

void StitchedCloud::registerWithSAC(PointCloudT::Ptr cloud, const int iters)
{
    // Transforms 'cloud' such that it more closely aligns with the stitched cloud
    // Sampled consensus initial alignment
    // Useful for fast but rough registration

    // Estimate normals in the cloud
    pcl::PointCloud<pcl::Normal>::Ptr src_normals (new pcl::PointCloud<pcl::Normal> ());
    pcl::PointCloud<pcl::Normal>::Ptr stitched_normals (new pcl::PointCloud<pcl::Normal> ());
    pcl::NormalEstimationOMP<PointT, pcl::Normal> normal_est;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    normal_est.setSearchMethod(tree);;
    normal_est.setKSearch(100);
    normal_est.setInputCloud(cloud);
    normal_est.compute(*src_normals);
    normal_est.setInputCloud(stitched_cloud);
    normal_est.compute(*stitched_normals);

    // Locate features using the normals and clouds
    pcl::FPFHEstimationOMP<PointT, pcl::Normal, pcl::FPFHSignature33> fpfh;
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr src_features (new pcl::PointCloud<pcl::FPFHSignature33> ());
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr stitched_features (new pcl::PointCloud<pcl::FPFHSignature33> ());
    fpfh.setSearchMethod(tree);
    fpfh.setKSearch(250);       // must be larger than value for normals
    fpfh.setInputCloud(cloud);
    fpfh.setInputNormals(src_normals);
    fpfh.compute(*src_features);
    fpfh.setInputCloud(stitched_cloud);
    fpfh.setInputNormals(stitched_normals);
    fpfh.compute(*stitched_features);

    // Use the features found to perform the alignment
    pcl::SampleConsensusInitialAlignment<PointT, PointT, pcl::FPFHSignature33> sac_ia;
    sac_ia.setMaximumIterations(iters);
    // Stitched
    sac_ia.setInputTarget(stitched_cloud);
    sac_ia.setTargetFeatures(stitched_features);
    // Un-registered cloud
    sac_ia.setInputSource(cloud);
    sac_ia.setSourceFeatures(src_features);
    sac_ia.align(*cloud);
}


/*          Helper functions        */
void removeOutliers(PointCloudT::Ptr cloud, const int num_neighbours, const int stddev)
{
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(num_neighbours);        // The number of neighbours to analyse
    sor.setStddevMulThresh(stddev);      // Standard deviation multiplier
    sor.filter(*cloud);
}

void downSample(PointCloudT::Ptr cloud, const int leaf_size)
{
    // Smaller leaf sizes makes the cloud more accurate but also signficantly slower
    pcl::VoxelGrid<PointT> grid;
    grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    grid.setInputCloud(cloud);
    grid.filter(*cloud);
}

void transform(PointCloudT::Ptr cloud, const TransformData& t)
{
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << t.dx, t.dy, t.dz;
    transform.rotate (Eigen::AngleAxisf (-t.rotx, Eigen::Vector3f::UnitX()));
    transform.rotate (Eigen::AngleAxisf (-t.roty, Eigen::Vector3f::UnitY()));
    transform.rotate (Eigen::AngleAxisf (-t.rotz, Eigen::Vector3f::UnitZ()));
    pcl::transformPointCloud(*cloud, *cloud, transform);
}

void filterRangeZ(PointCloudT::Ptr cloud, const double minZ, const double maxZ)
{
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(minZ, maxZ);
    pass.filter(*cloud);
}

void reconstructSurface(pcl::PointCloud<pcl::PointNormal>::Ptr mls_points, const PointCloudT::Ptr cloud, const double radius)
{
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    pcl::MovingLeastSquares<PointT, pcl::PointNormal> mls;
    mls.setComputeNormals(true);
    mls.setInputCloud(cloud);
    mls.setPolynomialFit(true);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(radius);
    mls.process(*mls_points);
}
