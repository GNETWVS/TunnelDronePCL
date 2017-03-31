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
pcl::PCLPointCloud2
rectangularThreshold(pcl::PCLPointCloud2::Ptr src_cloud, std::vector<double> thresh_range);

/*

Steps:
1. Filter longitudinally
2. Use RANSAC to fit planes to each of the four walls
2. Apply statistical outlier removal (?)

*/

int main (int argc, char** argv)
{
    // pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr stitched_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2::Ptr src_cloud (new pcl::PCLPointCloud2 ());
    pcl::PCLPointCloud2::Ptr filtered_cloud (new pcl::PCLPointCloud2 ());
    pcl::PCLPointCloud2::Ptr stitched_cloud (new pcl::PCLPointCloud2 ());

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
        // reader.read<pcl::PointXYZ> (path, *src_cloud);
        reader.read(path, *src_cloud);


        /*          Passthrough filter          */
        // Depth filter
        // pcl::PassThrough<pcl::PointXYZ> pass;
        pcl::PassThrough<pcl::PCLPointCloud2> pass;
        pass.setInputCloud(src_cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(-5, 0);
        pass.filter(*filtered_cloud);

        // Fit a plane to each of the 4 sides of the tunnel
        *filtered_cloud = rectangularThreshold(filtered_cloud, {-5,0,5,-5,0,5});

        // Add the filtered points to the stitched cloud
        *stitched_cloud = *filtered_cloud;



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
    }
    // TODO: Convert from pointcloudXYZ to pointcloud2
    // pcl::PCLPointCloud2::Ptr stitched_cloud_2 (new pcl::PCLPointCloud2());
    // pcl::copyPointCloud(stitched_cloud, stitched_cloud_2);
    // pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    // sor.setInputCloud(stitched_cloud);
    // sor.setLeafSize(0.01, 0.01, 0.01);
    // sor.filter(*stitched_cloud);

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

    pcl::PCDWriter writer;
    std::string filename = directory + "filtered.pcd";
    writer.write(filename, *stitched_cloud, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), false);

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
pcl::PCLPointCloud2
rectangularThreshold(pcl::PCLPointCloud2::Ptr src_cloud, std::vector<double> thresh_range)
{
    /*          I/O         */
    // returns a point cloud of 4 orthogonal planes
    // thresh_range should be in the form {min_x, mid_x, max_x, min_y, mid_y, max_y}

    const double thresh = 0.1;
    pcl::PCLPointCloud2::Ptr tmp_cloud (new pcl::PCLPointCloud2);
    pcl::PCLPointCloud2::Ptr ret_cloud (new pcl::PCLPointCloud2);


    for (int i = 0; i < 4; ++i)
    {
        std::vector<int> inliers;

        // Filter the input cloud into left, right, upper and lower sections
        pcl::PCLPointCloud2::Ptr sub_cloud (new pcl::PCLPointCloud2);
        pcl::PassThrough<pcl::PCLPointCloud2> pass;
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

        // WHY THE FUCK DOESN'T RANSAC SUPPORT PCLPOINTCLOUD2????

        // Use RANSAC to fit a plane to the thresholded points
        pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr
            sub_plane (new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB> (sub_cloud));
        pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac (sub_plane);
        ransac.setDistanceThreshold(thresh);
        ransac.computeModel();
        ransac.getInliers(inliers);
        pcl::copyPointCloud<pcl::PCLPointCloud2>(*sub_cloud, inliers, *tmp_cloud);
        *ret_cloud += *tmp_cloud;
    }

    return *ret_cloud;
}
