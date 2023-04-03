cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM3_test)
add_definitions(-w)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

################ CHANGE THIS ######################
SET(ORB_SLAM_DIR ORB_SLAM3)
SET(LITAMIN_DIR litamin_ws/src/litamin2)
###################################################

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

# Check C++14 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++14.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
endif()
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# include(${CMAKE_CURRENT_SOURCE_DIR}/ORB_SLAM3/CMakeLists.txt)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/out)
find_package(OpenCV 4.4)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 4.4 not found.")
   endif()
MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})
# Litamin2
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED QUIET)
find_package(Pangolin REQUIRED)
find_package(realsense2)
find_package(OpenMP)
find_package(Eigen3 3.1.0 REQUIRED)


if (OPENMP_FOUND)
	message("OPENMP FOUND")
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#	set(CMAKE_C_FLAGS "$(CMAKE_C_FLAGS) $(OpenMP_C_FLAGS)")
#	set(CMAKE_CXX_FLAGS "$(CMAKE_CXX_FLAGS) $(OpenMP_CXX_FLAGS)")
endif()

include_directories(
	${ORB_SLAM_DIR}
	${ORB_SLAM_DIR}/include
	${ORB_SLAM_DIR}/include/CameraModels
	${ORB_SLAM_DIR}/Thirdparty/Sophus
	${Pangolin_INCLUDE_DIRS}
)

include_directories(
	${LITAMIN_DIR}/include
	${PCL_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
)

add_library(litamin SHARED
	${LITAMIN_DIR}/src/litamin2/litamin2.cpp
	${LITAMIN_DIR}/src/fast_gicp/gicp/lsq_registration.cpp
	${LITAMIN_DIR}/src/litamin2/litamin2point2voxel.cpp
	${LITAMIN_DIR}/src/litamin2/litamin2point2voxelnewton.cpp
)
target_link_libraries(litamin
	${PCL_LIBRARIES}
 	${CERES_LIBRARIES}	
)
target_include_directories(litamin PUBLIC
	${LITAMIN_DIR}/include
	${PCL_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${ORB_SLAM_DIR}/lib)

add_library(ORB_SLAM SHARED
${ORB_SLAM_DIR}/src/System.cc
${ORB_SLAM_DIR}/src/Tracking.cc
${ORB_SLAM_DIR}/src/LocalMapping.cc
${ORB_SLAM_DIR}/src/LoopClosing.cc
${ORB_SLAM_DIR}/src/ORBextractor.cc
${ORB_SLAM_DIR}/src/ORBmatcher.cc
${ORB_SLAM_DIR}/src/FrameDrawer.cc
${ORB_SLAM_DIR}/src/Converter.cc
${ORB_SLAM_DIR}/src/MapPoint.cc
${ORB_SLAM_DIR}/src/KeyFrame.cc
${ORB_SLAM_DIR}/src/Atlas.cc
${ORB_SLAM_DIR}/src/Map.cc
${ORB_SLAM_DIR}/src/MapDrawer.cc
${ORB_SLAM_DIR}/src/Optimizer.cc
${ORB_SLAM_DIR}/src/Frame.cc
${ORB_SLAM_DIR}/src/KeyFrameDatabase.cc
${ORB_SLAM_DIR}/src/Sim3Solver.cc
${ORB_SLAM_DIR}/src/Viewer.cc
${ORB_SLAM_DIR}/src/ImuTypes.cc
${ORB_SLAM_DIR}/src/G2oTypes.cc
${ORB_SLAM_DIR}/src/CameraModels/Pinhole.cpp
${ORB_SLAM_DIR}/src/CameraModels/KannalaBrandt8.cpp
${ORB_SLAM_DIR}/src/OptimizableTypes.cpp
${ORB_SLAM_DIR}/src/MLPnPsolver.cpp
${ORB_SLAM_DIR}/src/GeometricTools.cc
${ORB_SLAM_DIR}/src/TwoViewReconstruction.cc
${ORB_SLAM_DIR}/src/Config.cc
${ORB_SLAM_DIR}/src/Settings.cc
${ORB_SLAM_DIR}/include/System.h
${ORB_SLAM_DIR}/include/Tracking.h
${ORB_SLAM_DIR}/include/LocalMapping.h
${ORB_SLAM_DIR}/include/LoopClosing.h
${ORB_SLAM_DIR}/include/ORBextractor.h
${ORB_SLAM_DIR}/include/ORBmatcher.h
${ORB_SLAM_DIR}/include/FrameDrawer.h
${ORB_SLAM_DIR}/include/Converter.h
${ORB_SLAM_DIR}/include/MapPoint.h
${ORB_SLAM_DIR}/include/KeyFrame.h
${ORB_SLAM_DIR}/include/Atlas.h
${ORB_SLAM_DIR}/include/Map.h
${ORB_SLAM_DIR}/include/MapDrawer.h
${ORB_SLAM_DIR}/include/Optimizer.h
${ORB_SLAM_DIR}/include/Frame.h
${ORB_SLAM_DIR}/include/KeyFrameDatabase.h
${ORB_SLAM_DIR}/include/Sim3Solver.h
${ORB_SLAM_DIR}/include/Viewer.h
${ORB_SLAM_DIR}/include/ImuTypes.h
${ORB_SLAM_DIR}/include/G2oTypes.h
${ORB_SLAM_DIR}/include/CameraModels/GeometricCamera.h
${ORB_SLAM_DIR}/include/CameraModels/Pinhole.h
${ORB_SLAM_DIR}/include/CameraModels/KannalaBrandt8.h
${ORB_SLAM_DIR}/include/OptimizableTypes.h
${ORB_SLAM_DIR}/include/MLPnPsolver.h
${ORB_SLAM_DIR}/include/GeometricTools.h
${ORB_SLAM_DIR}/include/TwoViewReconstruction.h
${ORB_SLAM_DIR}/include/SerializationUtils.h
${ORB_SLAM_DIR}/include/Config.h
${ORB_SLAM_DIR}/include/Settings.h)

target_link_libraries(ORB_SLAM
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}

###################### CHANGE THIS ###################
/home/alexmano/Desktop/NA568/ORB_SLAM3/Thirdparty/DBoW2/lib/libDBoW2.so
/home/alexmano/Desktop/NA568/ORB_SLAM3/Thirdparty/g2o/lib/libg2o.so
######################################################

-lboost_serialization
-lcrypto
)

# target_link_libraries(ORB_SLAM
# ${OpenCV_LIBS}
# ${EIGEN3_LIBS}
# ${Pangolin_LIBRARIES}
# ${ORB_SLAM_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
# ${ORB_SLAM_DIR}/Thirdparty/g2o/lib/libg2o.so
# -lboost_serialization
# -lcrypto
# )

add_subdirectory(${ORB_SLAM_DIR}/Thirdparty/g2o)

# Executables
add_executable(test_out_thing
        test.cc)

target_link_libraries(test_out_thing 
        ORB_SLAM
        ${catkin_LIBRARIES}
        ${CERES_LIBRARIES}
        ${PCL_LIBRARIES}
        litamin   
)