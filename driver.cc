/*
Driver for combination of ORBSLAM3 and Litamin2 integration

usage: ./driver [single/double] path_to_vocabulary path_to_settings path_to_sequence path_to_kitti_dataset dataset_type out_path
*/


#include <signal.h>


#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <ctime>
#include <sstream>
#include <string>

#include <condition_variable>

#include <opencv2/core/core.hpp>

#include <librealsense2/rs.hpp>
#include "librealsense2/rsutil.h"


#include <System.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>


#include <fast_gicp/gicp/fast_gicp.hpp>
#include "fast_gicp/gicp/impl/fast_gicp_impl.hpp"
#include <litamin2/litamin2point2voxel.hpp>

using namespace std;

void load_images(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps);

int main(int argc, char **argv) {

    // Get arguments
    if(argc != 8) {
        cerr << "INCORRECT NUMBER OF ARGUMENTS\nUSAGE: ./driver [single/double] path_to_vocabulary path_to_settings 
                path_to_sequence path_to_kitti_dataset dataset_type out_path\n";
        return 1;
    }
    string mode = argv[1];
    string path_to_vocabulary = argv[2];
    string path_to_settings = argv[3];

    // Load images for ORBSLAM
    vector<string> vstr_image_left;
    vector<string> vstr_image_right;
    vector<double> v_timestamps;
    string path_to_sequence = argv[4];
    load_images(path_to_sequence, vstr_image_left, vstr_image_right, v_timestamps);

    const int n_images = vstr_image_left.size();

    // Create SLAM system
    ORB_SLAM3::System SLAM(path_to_vocabulary, path_to_settings, ORB_SLAM3::System::STEREO, true);
    float image_scale = SLAM.GetImageScale();

    vector<float> v_times_track;
    v_times_track.resize(n_images);
    double t_track = 0.f;
    double t_resize = 0.f;

    // TODO setup litamin
    DatasetOptions dataset_options;
    dataset_options.root_path = argv[5];
    strign dataset_type = argv[6];
    if(dataset_type == "KITI_raw") {
        dataset_options.dataset = KITTI_raw;
    } else if(dataset_type == "KITTI") {
        dataset_options.dataset = KITTI;
    }

    string out_path = argv[7];
    auto sequences = get_sequences(dataset_options);
    int num_sequences = (int) sequences.size(); // TODO might only be one sequence??

    // Check size of number of elements
    if(num_sequences != n_images) {
        cerr << "Number of images and lidar data does not match!" << endl;
        return 1;
    }

    // use downsample_resolution=1.0 for fast registration
    double downsample_resolution = 0.25;
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> voxelgrid;
    voxelgrid.setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);


    litamin::LiTAMIN2Point2Voxel<pcl::PointXYZ, pcl::PointXYZ> litamin2;
    // litamin::LiTAMIN2Point2VoxelNewton<pcl::PointXYZ, pcl::PointXYZ> litamin2;
    litamin2.setNumThreads(4);
    litamin2.setResolution(3.0);
    litamin2.setMaxCorrespondenceDistance(1.0);
    litamin2.setTransformationEpsilon(1e-2);
    litamin2.setMaximumIterations(64);

    // trajectory for visualization
    pcl::PointCloud<pcl::PointXYZ>::Ptr trajectory(new pcl::PointCloud<pcl::PointXYZ>);
    trajectory->push_back(pcl::PointXYZ(0.0f, 0.0f, 0.0f));

    pcl::visualization::PCLVisualizer vis;
    vis.setBackgroundColor(0, 0, 0);
    vis.addCoordinateSystem(1.0);
    vis.addText("KITTI trajectory pcl_visualizer", 10, 10, "debugger text", 0);
    vis.initCameraParameters();
    // position x,y,z         view x,y,z      view up: x,y,z
    vis.setCameraPosition(99.8088, 142.249, 533.837,  177.075, 20.2209, 21.9058,  0.986978, 0.101531, 0.124763);
    vis.setCameraClipDistances(519.902, 554.931);  
    vis.addPointCloud<pcl::PointXYZ>(trajectory, "trajectory");

    // Litamin loop prep
    int sequence_id = sequences[0].sequence_id;
    auto iterator_ptr = get_dataset_sequence(dataset_options, sequence_id);

    iterator_ptr->SetInitFrame(0);
    iterator_ptr->printDatasetType(dataset_options);

    // set initial frame as target
    int frame_id = 0;
    voxelgrid.setInputCloud(iterator_ptr->Frame(frame_id).makeShared());
    pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
    voxelgrid.filter(*target);
    litamin2.setInputTarget(target);

    // sensor pose sequence
    int seq_size = iterator_ptr->NumFrames();
    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses(seq_size);
    poses[0].setIdentity();

    // Check size of number of elements
    if(seq_size != n_images) {
        cerr << "Number of images and lidar data does not match!" << endl;
        return 1;
    }


    // Main loop
    cv::Mat im_left, im_right;

    Sophus::SE3f pose;

    for(int ni = 0; ni < n_images; ni++) {

        // Load images
        im_left = cv::imread(vstr_image_left[ni], cv::IMREAD_UNCHANGED);
        im_right = cv::imread(vstr_image_right[ni], cv::IMREAD_UNCHANGED);
        double tframe = v_timestamps[ni];

        if(im_left.empty()) {
            cerr << endl << "Failed to load image at: " << string(vstr_image_left[ni]) << endl;
            return 1;
        }
        if(im_right.empty()) {
            cerr << endl << "Failed to load image at: " << string(vstr_image_right[ni]) << endl;
            return 1;
        }

        if(image_scale != 1.f) {
#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t_Start_Resize = std::chrono::monotonic_clock::now();
    #endif
#endif
            int width = im_left.cols * image_scale;
            int height = im_left.rows * image_scale;
            cv::resize(im_left, im_left, cv::Size(width, height));
            cv::resize(im_right, im_right, cv::Size(width, height));

#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t_End_Resize = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t_End_Resize = std::chrono::monotonic_clock::now();
    #endif
            t_resize = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t_End_Resize - t_Start_Resize).count();
            SLAM.InsertResizeTime(t_resize);
#endif
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Litamin loop
        frame_id++;
        pcl::PointCloud<pcl::PointXYZ> frame = iterator_ptr->Next();
        

        if(mode == "single") {
            // Pass only to ORBLAM3->Litamin2
            pose_next = orbslam3_litamin2(pose, tframe, im_left, im_right, frame)

        } else if(mode == "double") {
            // Pass to ORBSLAM3->Litamin2 and Litamin2->ORBSLAM3
            pose_1 = orbslam3_litamin2(pose, tframe, im_left, im_right, frame)
            pose_2 = litmain2_orbslame3(pose, tframe, im_left, im_right, frame)

            // Do something with the poses
            pose_next = ...
        }

        // Update pose
        pose = pose_next;

        // TODO Visualization

    }
}

// Taken from example code
void load_images(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";
    string strPrefixRight = strPathToSequence + "/image_1/";

    const int nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
    }
}
