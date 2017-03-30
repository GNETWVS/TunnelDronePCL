#include <cstdlib>
#include <iostream>
#include <vector>
#include <string>
#include <sys/types.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

namespace fs = boost::filesystem;

std::vector<std::string> getFileList(const std::string& path);

/*

Steps:
1. Filter longitudinally
2. Apply statistical outlier removal

*/

int main (int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    /*          Handle Input        */
    // Check whether a file has been supplied
    if (argc < 3)
    {
        std::cout << "Input should be of the form:\n"
                  << "./cleanup_pcd -f <file> or ./cleanup_pcd -d <directory>" << std::endl;
        return -1;
    }

    std::vector<std::string> files_to_process;
    std::string directory;
    if (!std::strcmp(argv[1], "-f"))
    {
        std::string filename = argv[2];
        files_to_process.push_back(filename);
    }
    else if (!std::strcmp(argv[1], "-d"))
    {
        directory = argv[2];
        files_to_process = getFileList(directory);
    }
    else
    {
        std::cout << "Command \"" << argv[1] << "\" not recognised." << std::endl;
        return -1;
    }

    for (auto file : files_to_process)
    {
        std::size_t i = file.find(".");
        int len = file.size();
        if (len < 5 || file.substr(i, 4).compare(".pcd"))
        {
            // Not a PCD file
            continue;
        }


        // Read cloud data from the supplied file
        pcl::PCDReader reader;
        std::string path = directory + file;
        reader.read<pcl::PointXYZ> (path, *src_cloud);

        /*          Visualisation           */
        pcl::visualization::PCLVisualizer viewer ("PCD Filtering");
        // Source point cloud - WHITE
        // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_cloud_colour_handler (src_cloud, 255, 255, 255);
        // viewer.addPointCloud(src_cloud, src_cloud_colour_handler, "source_cloud");

        /*          Passthrough filter          */
        // Create point clouds for each of the 4 sides of the tunnel
        // Global threshold
        pcl::PassThrough<pcl::PointXYZ> pass_z;
        pass_z.setInputCloud(src_cloud);
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(-5, 0);
        pass_z.filter(*filtered_cloud);
        // Left side
        pcl::PointCloud<pcl::PointXYZ>::Ptr left_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass_left;
        pass_left.setInputCloud(filtered_cloud);
        pass_left.setFilterFieldName("x");
        pass_left.setFilterLimits(-5, 0);
        pass_left.filter(*left_cloud);
        // Right side
        pcl::PointCloud<pcl::PointXYZ>::Ptr right_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass_right;
        pass_right.setInputCloud(filtered_cloud);
        pass_right.setFilterFieldName("x");
        pass_right.setFilterLimits(0, 5);
        pass_right.filter(*right_cloud);
        // Up side
        pcl::PointCloud<pcl::PointXYZ>::Ptr up_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass_up;
        pass_up.setInputCloud(filtered_cloud);
        pass_up.setFilterFieldName("y");
        pass_up.setFilterLimits(0, 5);
        pass_up.filter(*up_cloud);
        // Down side
        pcl::PointCloud<pcl::PointXYZ>::Ptr down_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass_down;
        pass_down.setInputCloud(filtered_cloud);
        pass_down.setFilterFieldName("y");
        pass_down.setFilterLimits(-5, 0);
        pass_down.filter(*down_cloud);

        /*          Statistical outlier removal         */
        // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        // sor.setInputCloud(filtered_cloud);
        // sor.setMeanK(50);
        // sor.setStddevMulThresh(1.0);
        // sor.filter(*filtered_cloud);

        /*          Resampling - Not very effective          */
        // Create a KD tree
        // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        // pcl::PointCloud<pcl::PointNormal> mls_points;
        // pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
        // mls.setComputeNormals(true);
        // mls.setInputCloud(filtered_cloud);
        // mls.setPolynomialFit(false);
        // mls.setSearchMethod(tree);
        // mls.setSearchRadius(0.05);
        // mls.process(mls_points);
        // pcl::copyPointCloud(mls_points, *filtered_cloud);

        const double thresh = 0.1;
        /*          RANSAC Plane            */
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        std::vector<int> inliers;
        // Left
        pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
            left_plane (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (left_cloud));
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac_left (left_plane);
        ransac_left.setDistanceThreshold(thresh);
        ransac_left.computeModel();
        ransac_left.getInliers(inliers);
        pcl::copyPointCloud<pcl::PointXYZ>(*left_cloud, inliers, *filtered_cloud);
        inliers.clear();
        // Right
        pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
            right_plane (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (right_cloud));
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac_right (right_plane);
        ransac_right.setDistanceThreshold(thresh);
        ransac_right.computeModel();
        ransac_right.getInliers(inliers);
        pcl::copyPointCloud<pcl::PointXYZ>(*right_cloud, inliers, *tmp_cloud);
        *filtered_cloud += *tmp_cloud;
        inliers.clear();
        // Up
        pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
            up_plane (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (up_cloud));
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac_up (up_plane);
        ransac_up.setDistanceThreshold(thresh);
        ransac_up.computeModel();
        ransac_up.getInliers(inliers);
        pcl::copyPointCloud<pcl::PointXYZ>(*up_cloud, inliers, *tmp_cloud);
        *filtered_cloud += *tmp_cloud;
        inliers.clear();
        // Down
        pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
            down_plane (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (down_cloud));
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac_down (down_plane);
        ransac_down.setDistanceThreshold(thresh);
        ransac_down.computeModel();
        ransac_down.getInliers(inliers);
        pcl::copyPointCloud<pcl::PointXYZ>(*down_cloud, inliers, *tmp_cloud);
        *filtered_cloud += *tmp_cloud;
        inliers.clear();

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_colour_handler (src_cloud, 150, 6, 6);
        viewer.addPointCloud (src_cloud, src_colour_handler, "src_cloud");

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> filtered_point_colour_handler (filtered_cloud, 250, 250, 250);
        viewer.addPointCloud (filtered_cloud, filtered_point_colour_handler, "filtered_cloud");

        viewer.addCoordinateSystem (1.0, "cloud", 0);
        viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "src_cloud");
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "filtered_cloud");
        //viewer.setPosition(800, 400); // Setting visualiser window position

        while (!viewer.wasStopped ())
        {
            viewer.spinOnce ();
        }
    }
    return (0);
}

std::vector<std::string> getFileList(const std::string& path)
{
    std::vector<std::string> files;
    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(path.c_str())) == NULL) {
        std::cout << "Error opening " << path << std::endl;
        return files;
    }

    while ((dirp = readdir(dp)) != NULL) {
        files.push_back(std::string(dirp->d_name));
    }
    closedir(dp);
    return files;
}
