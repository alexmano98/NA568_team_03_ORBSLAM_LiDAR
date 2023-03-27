cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM3_test)

SET(ORB_SLAM_DIR ORB_SLAM3)
SET(LITAMIN_DIR litamin_ws/src/litamin2)

include(${CMAKE_CURRENT_SOURCE_DIR}/ORB_SLAM3/CMakeLists.txt)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/out)

# Litamin2
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED QUIET)
find_package(OpenMP)

if (OPENMP_FOUND)
	message("OPENMP FOUND")
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#	set(CMAKE_C_FLAGS "$(CMAKE_C_FLAGS) $(OpenMP_C_FLAGS)")
#	set(CMAKE_CXX_FLAGS "$(CMAKE_CXX_FLAGS) $(OpenMP_CXX_FLAGS)")
endif()

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


# Executables
add_executable(test_out_thing
        test.cc)

target_link_libraries(test_out_thing 
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${CERES_LIBRARIES}
        ${PCL_LIBRARIES}
        litamin   
)