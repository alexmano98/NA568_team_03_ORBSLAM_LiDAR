/*
Driver for combination of ORBSLAM3 and Litamin2 integration

usage: ./driver [single/double] path_to_vocabulary path_to_settings path_to_sequence path_to_kitti_dataset dataset_type slam_file litamin_file output_loc
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

#include "dataloader/dataset.hpp"
#include <fast_gicp/gicp/fast_gicp.hpp>
#include "fast_gicp/gicp/impl/fast_gicp_impl.hpp"
#include <litamin2/litamin2point2voxel.hpp>
#include "litamin2/litamin2point2voxelnewton.hpp"


using namespace litamin;
using namespace std;


Sophus::SE3f orbslam3_litamin2(ORB_SLAM3::System &SLAM, const string &litamin_file, const cv::Mat &im_left, const cv::Mat &im_right, const double &tframe);

Sophus::SE3f litamin2_orbslam3(const string &slam_file, auto &iterator_ptr, vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> &poses,
    litamin::LiTAMIN2Point2Voxel<pcl::PointXYZ, pcl::PointXYZ> &litamin2, pcl::ApproximateVoxelGrid<pcl::PointXYZ> &voxelgrid, pcl::visualization::PCLVisualizer &vis, 
     pcl::PointCloud<pcl::PointXYZ>::Ptr &trajectory, int frame_id);

void load_images(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &v_timestamps);

int main(int argc, char **argv) {

    // Get arguments
    if(argc != 10) {
        cerr << "INCORRECT NUMBER OF ARGUMENTS\nUSAGE: ./driver [single/double] path_to_vocabulary path_to_settings path_to_sequence path_to_kitti_dataset dataset_type slam_file litamin_file output_loc\n";
        return 1;
    }

    string slam_file = argv[7];
    string litamin_file = argv[8];
    string out_file = argv[9];

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
    string dataset_type = argv[6];
    if(dataset_type == "KITI_raw") {
        dataset_options.dataset = litamin::KITTI_raw;
    } else if(dataset_type == "KITTI") {
        dataset_options.dataset = litamin::KITTI;
    }

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

    // Read in slam_file
    //****************************************************************************************************
    // Eigen::Isometry3d a;
    
    ifstream infile(slam_file);
      // read txt file

    string line;
    int index = 0;
    while (getline(infile, line))
    {
        // Eigen::Isometry3d temp;
        Eigen::Isometry3d T1=Eigen::Isometry3d::Identity();
        // store each line into row 
        vector<double> row;
        for (int i = 0; i < 12; i++)
        {
            double num;
            infile >> num;
            row.push_back(num);
            cout<<num<<endl;

        }
        
        vector<vector<double>> matrix(3, vector<double>(4));
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                //matrix[i][j] = row[i * 4 + j];
                T1(i,j) = row[i*4+j];
            }
        }

        // poses.push_back(matrix);
        poses[index] = T1;
        cout<<poses[index](0,0);
        index++;
    }
;
//****************************************************************************************************



    // Main loop
    cv::Mat im_left, im_right;

    Sophus::SE3f pose;
    std::vector<Sophus::SE3f> out_poses(seq_size);

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
        
        Sophus::SE3f pose_next;

        if(mode == "single") {
            // Pass only to ORBLAM3->Litamin2
            pose_next = orbslam3_litamin2(SLAM, litamin_file, im_left, im_right, tframe);

        } else if(mode == "double") {
            // Pass to ORBSLAM3->Litamin2 and Litamin2->ORBSLAM3
            Sophus::SE3f pose_1 = orbslam3_litamin2(SLAM, litamin_file, im_left, im_right, tframe);
            Sophus::SE3f pose_2 = litamin2_orbslam3(slam_file, iterator_ptr, poses, litamin2, voxelgrid, vis, trajectory, ni);

            // Average poses
            auto translation_matrix = (pose_1.translation() + pose_2.translation()) / 2;
            // auto translation_matrix = pose_1.translation();
            auto rotation_matrix = pose_1.rotationMatrix(); // There is no easy way to average rotations, so just use one of them
            pose_next = Sophus::SE3f(rotation_matrix, translation_matrix);
        }

        // Update pose
        pose = pose_next;
        out_poses[ni] = pose;

        // time

        // Visualization litamin
        trajectory->push_back(pcl::PointXYZ(poses[frame_id](0, 3), poses[frame_id](1, 3), poses[frame_id](2, 3)));
        vis.updatePointCloud<pcl::PointXYZ>(trajectory, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(trajectory, 255.0, 0.0, 0.0), "trajectory");
        vis.spinOnce();   

        // time
        #ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

#ifdef REGISTER_TIMES
        t_track = t_resize + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
        SLAM.InsertTrackTime(t_track);
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        v_times_track[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<n_images-1)
            T = v_timestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-v_timestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);

    // TODO set this with argument/filesize or is this even necessary
        // if(ni == 800) {
        //     break;
        // }  


    }

    // save the estimated poses
    std::string out_file_path;
    std::stringstream ss;
    ss << std::setw(2) << std::setfill('1') << sequence_id << ".txt";
    out_file_path = out_file;
    cout << out_file_path << endl;
    std::ofstream ofs(out_file_path);
    for (const auto& pose : poses) {
      for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
          if (i || j) {
            ofs << " ";
          }

          ofs << pose(i, j);
        }
      }
      ofs << std::endl;        
    }

    return 0;
}

// Taken from example code
void load_images(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &v_timestamps)
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
            v_timestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";
    string strPrefixRight = strPathToSequence + "/image_1/";

    const int nTimes = v_timestamps.size();
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

Sophus::SE3f orbslam3_litamin2(ORB_SLAM3::System &SLAM, const string &litamin_file, const cv::Mat &im_left, const cv::Mat &im_right, const double &tframe) {
    return SLAM.TrackStereo(true, litamin_file, im_left, im_right, tframe);
}

Sophus::SE3f litamin2_orbslam3(const string &slam_file, auto &iterator_ptr, vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> &poses,
    litamin::LiTAMIN2Point2Voxel<pcl::PointXYZ, pcl::PointXYZ> &litamin2, pcl::ApproximateVoxelGrid<pcl::PointXYZ> &voxelgrid, pcl::visualization::PCLVisualizer &vis,
     pcl::PointCloud<pcl::PointXYZ>::Ptr &trajectory, int frame_id) {
    pcl::PointCloud<pcl::PointXYZ> frame = iterator_ptr->Next();
    // cout << "Seq.Frame_id: [" << sequence_id << "] - " << frame_id << endl; 
        // cout << "frame pts num: " << frame.size() << endl;

    // set the current frame as source
    voxelgrid.setInputCloud(frame.makeShared());
    pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
    voxelgrid.filter(*source);
    litamin2.setInputSource(source);

    // align and swap source and target cloud for next registration
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>);
    litamin2.align(*aligned);      
    litamin2.swapSourceAndTarget();

    // accumulate pose
    poses[frame_id] = poses[frame_id - 1] * litamin2.getFinalTransformation().cast<double>();

    //   for (int i = 0; i < 3; i++) {
    //     for (int j = 0; j < 4; j++) {
    //       // if (i || j) {
    //         cout << poses[frame_id](i,j)<<" ";
    //       // }

    //       // ofs << pose(i, j);
    //     }
    //     cout << endl;
    //   }

    // visualization
    trajectory->push_back(pcl::PointXYZ(poses[frame_id](0, 3), poses[frame_id](1, 3), poses[frame_id](2, 3)));
    vis.updatePointCloud<pcl::PointXYZ>(trajectory, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(trajectory, 255.0, 0.0, 0.0), "trajectory");
    vis.spinOnce();

    // Convert pose to SE3f
    Eigen::Matrix<float, 3, 3> R = poses[frame_id].rotation().cast<float>();
    Eigen::Matrix<float, 3, 1> t = poses[frame_id].translation().cast<float>();
    // Sophus::SE3f pose(R, t);
    return Sophus::SE3f(R,t);
}
