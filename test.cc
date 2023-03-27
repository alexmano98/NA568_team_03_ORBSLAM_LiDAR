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

    std::cout << "Hello World!\n";
}