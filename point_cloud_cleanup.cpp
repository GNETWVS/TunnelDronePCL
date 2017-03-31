#include <cstdlib>
#include <iostream>
#include <vector>
#include <string>
#include <sys/types.h>

#include <pcl/conversions.h>
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
#include <pcl/filters/voxel_grid.h>

namespace fs = boost::filesystem;


std::vector<std::string> getFileList(const std::string& path);
pcl::PointCloud<pcl::PointXYZ>
rectangularThreshold(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud, std::vector<double> thresh_range);

// TODO
// - Deal with potential culverts
// - Deal with tunnels

int main (int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr stitched_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

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

    // Remove non pcd files
    for (auto file = files_to_process.begin(); file < files_to_process.end(); ++file)
    {
        std::size_t i = file->find(".");
        int len = file->size();
        if (len < 5 || file->substr(i, 4).compare(".pcd"))
        {
            // Not a PCD file
            files_to_process.erase(file);
        }
    }

    int num_files = files_to_process.size();
    int  j = 0;
    for (auto file : files_to_process)
    {

        // Read cloud data from the supplied file
        pcl::PCDReader reader;
        std::string path = directory + file;
        // reader.read<pcl::PointXYZ> (path, *src_cloud);
        reader.read(path, *src_cloud);


        /*          Passthrough filter          */
        // Depth filter
        // pcl::PassThrough<pcl::PointXYZ> pass;
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(src_cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(-5, 0);
        pass.filter(*filtered_cloud);

        // Fit a plane to each of the 4 sides of the tunnel
        *filtered_cloud = rectangularThreshold(filtered_cloud, {-5,0,5,-5,0,5});

        // Add the filtered points to the stitched cloud
        *stitched_cloud += *filtered_cloud;
        std::cout << ++j << "/" << num_files << std::endl;

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
    }

    /*          Statistical outlier removal         */
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(stitched_cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(2.0);
    sor.filter(*stitched_cloud);

    /*          Downsampling            */
    pcl::PCLPointCloud2::Ptr stitched_cloud_2 (new pcl::PCLPointCloud2());
    // PointCloud -> PCLPointCloud2
    pcl::toPCLPointCloud2(*stitched_cloud, *stitched_cloud_2);
    pcl::VoxelGrid<pcl::PCLPointCloud2> vox_grid;
    vox_grid.setInputCloud(stitched_cloud_2);
    vox_grid.setLeafSize(0.1, 0.1, 0.1);
    vox_grid.filter(*stitched_cloud_2);

    // pcl::visualization::PCLVisualizer viewer ("PCD Filtering");

    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> stitched_colour_handler (stitched_cloud, 255, 255, 255);
    // viewer.addPointCloud (stitched_cloud, stitched_colour_handler, "src_cloud");
    //
    // viewer.addCoordinateSystem (1.0, "cloud", 0);
    // viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    // viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "src_cloud");
    //
    // while (!viewer.wasStopped ())
    // {
    //     viewer.spinOnce ();
    // }

    std::string filename = directory + "filtered.pcd";
    pcl::PCDWriter writer;
    writer.write(filename, *stitched_cloud_2, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

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


/*          RANSAC Plane            */
pcl::PointCloud<pcl::PointXYZ>
rectangularThreshold(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud, std::vector<double> thresh_range)
{
    /*          I/O         */
    // returns a point cloud of 4 orthogonal planes
    // thresh_range should be in the form {min_x, mid_x, max_x, min_y, mid_y, max_y}

    const double thresh = 0.1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ret_cloud (new pcl::PointCloud<pcl::PointXYZ>);


    for (int i = 0; i < 4; ++i)
    {
        std::vector<int> inliers;

        // Filter the input cloud into left, right, upper and lower sections
        pcl::PointCloud<pcl::PointXYZ>::Ptr sub_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(src_cloud);
        if (i < 2)
        {
            pass.setFilterFieldName("x");
            pass.setFilterLimits(thresh_range[i], thresh_range[i+1]);
        }
        else
        {
            pass.setFilterFieldName("y");
            pass.setFilterLimits(thresh_range[i+1], thresh_range[i+2]);
        }
        pass.filter(*sub_cloud);

        // Use RANSAC to fit a plane to the thresholded points
        pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
            sub_plane (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (sub_cloud));
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (sub_plane);
        ransac.setDistanceThreshold(thresh);
        ransac.computeModel();
        ransac.getInliers(inliers);
        pcl::copyPointCloud<pcl::PointXYZ>(*sub_cloud, inliers, *tmp_cloud);
        *ret_cloud += *tmp_cloud;
    }
    return *ret_cloud;
}
