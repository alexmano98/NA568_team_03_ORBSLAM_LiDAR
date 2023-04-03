/*
Test for compilation
*/

#include <signal.h>


#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <ctime>
#include <sstream>

#include <condition_variable>

#include <opencv2/core/core.hpp>

#include <librealsense2/rs.hpp>
#include "librealsense2/rsutil.h"

#include <Eigen/Core>
#include <Eigen/SVD>

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

int main(int argc, char **argv) {

    std::cout << "Hello World2!\n";

    Eigen::Matrix<float, 3, 3> R;
    R << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    Eigen::Matrix<float, 3, 1> t;
    t << 1, 1, 1;
    // cout << R << endl;
    // cout << t << endl;
    Sophus::SE3f pose1(R, t);

    Eigen::Matrix<float, 3, 3> R2;
    R2 << 2, 2, 2, 2, 2, 2, 2, 2, 2;
    Eigen::Matrix<float, 3, 1> t2;
    t2 << 2, 2, 2;
    // cout << t << endl;
    Sophus::SE3f pose2(R, t2);

    Eigen::JacobiSVD<Eigen::Matrix<float,3,3>> svd(R);
    auto U = svd.matrixU();
    auto V = svd.matrixV();
    auto Q = U * V.transpose();
    cout << U << V << endl;
    cout << Q << endl;

    cout << (pose1.translation() + pose2.translation()) / 2 << endl;
    cout << pose1.rotationMatrix() << endl;

    // Eigen::Isometry3d pose(R,t);
    // cout << (R + R2) / 2 << endl;

    // Sophus::SE3f pose_next;

            // translation_matrix = vector<
}