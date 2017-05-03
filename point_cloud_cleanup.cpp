#include <cstdlib>
#include <iostream>
#include <vector>
#include <string>
#include <sys/types.h>
#include <iomanip>
#include <mutex>
#include <thread>
#include <fstream>
#include <sstream>
#include <algorithm>

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

#include <boost/make_shared.hpp>

namespace fs = boost::filesystem;

typedef pcl::PointXYZRGB PointT;

std::vector<std::string> getFileList(const std::string& path);
static bool filePredicate(const std::string &s);
void transformCloud(pcl::PointCloud<PointT>::Ptr src_cloud, double rot_x, double rot_y, double rot_z,
                        double trans_x, double trans_y, double trans_z);
void alignClouds(pcl::PointCloud<PointT>::Ptr src_cloud,
                 pcl::PointCloud<PointT>::Ptr stitched_cloud, int num_iters);

// TODO: Reduce the scope of these global variables
const int point_scale = 1000;
std::mutex stitch_mutex;
std::vector<std::vector<double> > translation_and_rotation;
bool transformations_file_supplied = false;

// TODO:
// Find the start of the tunnel without external input
// Rotate different point clouds to improve stitching

/*          Method overview         */
// 1. Remove outliers
// 2. Downsample
// 3. Shift origin to the start of the tunnel cloud
// 4. Remove points beyond 10m
// 5. Align cloud with its pair - the current stitched cloud
// 6. Stitch the pair together
// 7. Downsample the result
// 8. Repeat until all files have been stitched

// OPTIONAL: Perform this algorithm on subsets of the total number of files to accelerate the process

class MyPointRepresentation : public pcl::PointRepresentation<pcl::PointNormal>
{
    using pcl::PointRepresentation<pcl::PointNormal>::nr_dimensions_;
    public:
        MyPointRepresentation ()
        {
            nr_dimensions_ = 4;
        }

        virtual void copyToFloatArray (const pcl::PointNormal &p, float * out) const
        {
            out[0] = p.x;
            out[1] = p.y;
            out[2] = p.z;
            out[3] = p.curvature;
        }
};

void helpMessage()
{
    std::cout << "Usage:\n"
              << "\t-f <file>" << "\t\tProcess a single file.\n"
              << "\t-d <directory>" << "\t\tProcess all of the pcd files in a directory.\n"
              << "\t-t <txt file>" << "\t\tSupply translation and rotation information. (OPTIONAL)"
              << std::endl;
}

int main (int argc, char** argv)
{
    /*          Handle Input        */
    // Check whether a file has been supplied
    if (argc < 3)
    {
        helpMessage();
        return -1;
    }

    std::vector<std::string> files_to_process;
    std::string directory;
    if (!std::strcmp(argv[1], "-f"))
    {
        std::string filename = argv[2];
        int idx = filename.find_last_of("/");
        directory = filename.substr(0, idx);
        files_to_process.push_back(filename.substr(idx));
    }
    else if (!std::strcmp(argv[1], "-d"))
    {
        directory = argv[2];
        if (directory.back() != '/')
        {
            helpMessage();
            return -1;
        }
        files_to_process = getFileList(directory);
    }
    else
    {
        std::cout << "Command \"" << argv[1] << "\" not recognised." << std::endl;
        helpMessage();
        return -1;
    }

    // Read transformation information from a supplied file
    // In the order dx, dy, dz, rotx, roty, rotz
    std::vector<std::vector<double> > translation_and_rotation_raw;
    if (argc > 4 && !std::strcmp(argv[3], "-t"))
    {
        transformations_file_supplied = true;
        std::cout << "Using position information supplied by " << argv[4] << "." << std::endl;
        std::string line;
        std::ifstream infile(argv[4]);
        const int rows_to_skip = 2;
        const int cols_to_skip = 2;
        int row = -1;
        while (std::getline(infile, line))
        {
            // Skip header rows
            if (rows_to_skip > ++row)
            {
                continue;
            }

            translation_and_rotation_raw.push_back({});

            std::istringstream iss (line);
            double val = 0.0;
            int col = 0;
            while (iss >> val)
            {
                if (cols_to_skip > col++)
                {
                    continue;
                }
                translation_and_rotation_raw[row - rows_to_skip].push_back(val);
            }
        }
    }

    // Remove non pcd files
    files_to_process.erase(
        std::remove_if(files_to_process.begin(), files_to_process.end(), filePredicate), files_to_process.end());
    if (files_to_process.size() == 0)
    {
        std::cout << "No PCD files found." << std::endl;
        return -1;
    }

    // Sort files_to_process into ascending order
    struct compareFileNamesStruct
    {
        bool operator() (std::string &a, std::string &b)
        {
            size_t a_idx_start = a.find_last_of("D") + 1;
            size_t a_idx_end = a.find(".");
            int a_int = stoi(a.substr(a_idx_start, a_idx_end - a_idx_start));

            size_t b_idx_start = b.find_last_of("D") + 1;
            size_t b_idx_end = b.find(".");
            int b_int = stoi(b.substr(b_idx_start, b_idx_end - b_idx_start));

            return a_int < b_int;
        }
    } compareFilenames;
    std::sort(files_to_process.begin(), files_to_process.end(), compareFilenames);

    // Read the transformations file
    if (transformations_file_supplied)
    {
        // Use the number of known pcd files to match translation and rotation values to each file
        int num_rows_per_file = translation_and_rotation_raw.size() / files_to_process.size();
        int num_cols = translation_and_rotation_raw[0].size();
        // Find the average translation and rotation value for each axis
        for (int row = 0; row < translation_and_rotation_raw.size(); ++row)
        {
            if (row % num_rows_per_file == 0)
            {
                translation_and_rotation.push_back({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
            }
            for (int col = 0; col < num_cols; ++col)
            {
                int new_row = row / (translation_and_rotation_raw.size() / num_rows_per_file);
                translation_and_rotation[new_row][col] += translation_and_rotation_raw[row][col];
            }
        }
        for (int row = 0; row < translation_and_rotation.size(); ++row)
        {
            for (int col = 0; col < num_cols; ++col)
            {
                translation_and_rotation[row][col] /= num_rows_per_file;
            }
        }
    }

    // TODO: Re-enable multithreading
    // /*          Processing          */
    // // Create 4 threads to process 4 files simultaneously
    // // - If there are fewer than 4 files, then make that many threads
    // // Once a thread returns, if there are still files to process then restart it on a new file
    // std::vector<std::thread> process_threads;
    // int i = 0;
    // while (i < 4 && i < files_to_process.size())
    // {
    //     // Create at most 4 threads
    //     std::string path = directory + files_to_process[i];
    //     process_threads.push_back(std::thread(processPCD, path, stitched_cloud));
    //     ++i;
    //     std::cout << "Processing: " << i << "/" << files_to_process.size() << std::endl;
    // }
    // while (i < files_to_process.size())
    // {
    //     process_threads[0].join();
    //     process_threads.erase(process_threads.begin());
    //
    //     // Since a thread has finished, create a new one
    //     std::string path = directory + files_to_process[i];
    //     process_threads.push_back(std::thread(processPCD, path, stitched_cloud));
    //     ++i;
    //     std::cout << "Processing: " << i << "/" << files_to_process.size() << std::endl;
    // }
    // for (auto t = process_threads.begin(); t < process_threads.end(); ++t)
    // {
    //     t->join();
    // }
    /*          Processing          */
    pcl::PointCloud<PointT>::Ptr stitched_cloud (new pcl::PointCloud<PointT> ());
    for (int i = 0; i < files_to_process.size(); ++i)
    {
        std::cout << int(100 * (i+1) / files_to_process.size()) << "%" << std::endl;

        // Read in a new point cloud
        pcl::PointCloud<PointT>::Ptr src_cloud (new pcl::PointCloud<PointT> ());
        std::string path = directory + files_to_process[i];
        pcl::PCDReader reader;
        reader.read(path, *src_cloud);

        // Find extrema
        PointT min_pt, max_pt;
        pcl::getMinMax3D(*src_cloud, min_pt, max_pt);

        // Shift the origin to the start of the tunnel
        transformCloud(src_cloud, 0, 0, 0, 0, 0, -min_pt.z);

        pcl::PassThrough<PointT> pass;
        pass.setInputCloud(src_cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.5, 5 * point_scale);
        pass.filter(*src_cloud);

        // Downsample both clouds
        pcl::VoxelGrid<PointT> grid;
        grid.setLeafSize(0.01*point_scale, 0.01*point_scale, 0.01*point_scale);
        grid.setInputCloud(src_cloud);
        grid.filter(*src_cloud);
        grid.setInputCloud(stitched_cloud);
        grid.filter(*stitched_cloud);

        // Remove outliers from the new cloud to be added
        // pcl::StatisticalOutlierRemoval<PointT> sor;
        // sor.setInputCloud(src_cloud);
        // sor.setMeanK(1000);
        // sor.setStddevMulThresh(3);
        // sor.filter(*src_cloud);

        // Align and stitch clouds
        if (i == 0)
        {
            *stitched_cloud = *src_cloud;
        }
        else
        {
            alignClouds(src_cloud, stitched_cloud, 100);
            *stitched_cloud += *src_cloud;
        }
    }

    // Remove outliers from final result
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(stitched_cloud);
    sor.setMeanK(1000);
    sor.setStddevMulThresh(0.5);
    sor.filter(*stitched_cloud);

    // Reconstruct the surface
    // pcl::PointCloud<pcl::PointNormal> mls_points;
    // pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    // pcl::MovingLeastSquares<PointT, pcl::PointNormal> mls;
    // mls.setComputeNormals(true);
    // mls.setInputCloud(stitched_cloud);
    // mls.setPolynomialFit(true);
    // mls.setSearchMethod(tree);
    // mls.setSearchRadius(0.03 * point_scale);
    // mls.process(mls_points);

    pcl::PCLPointCloud2::Ptr stitched_cloud_2 (new pcl::PCLPointCloud2());
    pcl::toPCLPointCloud2(*stitched_cloud, *stitched_cloud_2);

    std::string filename = directory + "/filtered.pcd";
    pcl::PCDWriter writer;
    writer.write(filename, *stitched_cloud_2, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);
    // pcl::io::savePCDFile (filename, mls_points);
    return 0;
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

static bool filePredicate(const std::string &s)
{
    std::size_t i = s.find(".");
    return (!s.compare(".") || !s.compare("..") || !s.compare("filtered.pcd") || s.substr(i, 4).compare(".pcd"));
}

/*          Transformations            */
void transformCloud(pcl::PointCloud<PointT>::Ptr src_cloud, double rot_x, double rot_y, double rot_z,
                        double trans_x, double trans_y, double trans_z)
{
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << trans_x, trans_y, trans_z;
    transform.rotate (Eigen::AngleAxisf (-rot_x, Eigen::Vector3f::UnitX()));
    transform.rotate (Eigen::AngleAxisf (-rot_y, Eigen::Vector3f::UnitY()));
    transform.rotate (Eigen::AngleAxisf (-rot_z, Eigen::Vector3f::UnitZ()));
    pcl::transformPointCloud(*src_cloud, *src_cloud, transform);
}

void alignClouds(pcl::PointCloud<PointT>::Ptr src_cloud,
                 pcl::PointCloud<PointT>::Ptr stitched_cloud, int num_iters)
{
    // Use ICP to transform src_cloud to be aligned with stitched_cloud

    std::cout << "Finding normals..." << std::endl;
    // Compute surface normals and curvature
    pcl::PointCloud<pcl::PointNormal>::Ptr src_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr stitched_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);

    pcl::NormalEstimation<PointT, pcl::PointNormal> normal_est;

    normal_est.setSearchMethod(tree);
    normal_est.setKSearch(100);
    // normal_est.setRadiusSearch(0.03 * 1000);

    normal_est.setInputCloud(src_cloud);
    normal_est.compute(*src_normals);
    pcl::copyPointCloud(*src_cloud, *src_normals);

    normal_est.setInputCloud(stitched_cloud);
    normal_est.compute(*stitched_normals);
    pcl::copyPointCloud(*stitched_cloud, *stitched_normals);

    std::cout << "Done" << std::endl;
    // std::cout << "FPFH..." << std::endl;
    // // Compute a FPFH for SAC initial alignment
    //
    // pcl::PointCloud<pcl::PFHSignature125>::Ptr src_features (new pcl::PointCloud<pcl::PFHSignature125>);
    // pcl::PointCloud<pcl::PFHSignature125>::Ptr stitched_features (new pcl::PointCloud<pcl::PFHSignature125>);
    // pcl::PFHEstimation<pcl::PointNormal, pcl::PointNormal, pcl::PFHSignature125> pfh;
    // pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    //
    // pfh.setInputNormals(src_normals);
    // pfh.setSearchMethod(tree2);
    // pfh.setKSearch(1000);
    // pfh.compute(*src_features);
    //
    // std::cout << *src_normals << std::endl;
    // std::cout << *src_features << std::endl;
    //
    // pfh.setInputNormals(stitched_normals);
    // pfh.setSearchMethod(tree2);
    // pfh.setKSearch(1000);
    // pfh.compute(*stitched_features);
    //
    // std::cout << "Done" << std::endl;
    // std::cout << "SAC_IA..." << std::endl;
    // // Perform an initial alignment with SAC
    // pcl::SampleConsensusInitialAlignment<PointT, PointT, pcl::FPFHSignature33> sac_ia;
    //
    // sac_ia.setMaxCorrespondenceDistance(1 * point_scale);
    // sac_ia.setMaximumIterations(1000);
    // sac_ia.setInputTarget(stitched_cloud);
    // sac_ia.setTargetFeatures(stitched_features);
    //
    // sac_ia.setInputSource(src_cloud);
    // sac_ia.setSourceFeatures(src_features);
    //
    // sac_ia.align(*stitched_cloud);
    // std::cout << "Done" << std::endl;
    std::cout << "ICP..." << std::endl;
    // Weight x,y,z,curvature equally
    MyPointRepresentation point_representation;
    float alpha[4] = {1.0, 1.0, 1.0, 1.0};
    point_representation.setRescaleValues(alpha);

    pcl::IterativeClosestPointNonLinear<pcl::PointNormal, pcl::PointNormal> reg;
    reg.setTransformationEpsilon(1e-9);

    reg.setMaxCorrespondenceDistance(10 * point_scale);
    reg.setPointRepresentation(boost::make_shared<const MyPointRepresentation> (point_representation));

    reg.setInputSource(src_normals);
    reg.setInputTarget(stitched_normals);

    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, stitched_to_src;
    pcl::PointCloud<pcl::PointNormal>::Ptr reg_result = src_normals;
    reg.setMaximumIterations(num_iters);

    src_normals = reg_result;
    reg.setInputSource(src_normals);
    reg.align(*reg_result);

    Ti *= reg.getFinalTransformation();

    if (fabs ((reg.getLastIncrementalTransformation() - prev).sum()) < reg.getTransformationEpsilon ())
    {
        reg.setMaxCorrespondenceDistance(reg.getMaxCorrespondenceDistance() - 0.01 * point_scale);
    }

    prev = reg.getLastIncrementalTransformation();

    stitched_to_src = Ti.inverse();
    pcl::transformPointCloud(*stitched_cloud, *stitched_cloud, stitched_to_src);
    std::cout << "Done" << std::endl;
}
