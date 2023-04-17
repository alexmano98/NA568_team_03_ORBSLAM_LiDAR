# NA568_team_03_ORBSLAM_LiDAR
NA 568 WN 23 Team 03 Final Project

First, install ORB-SLAM3 in the same directory as this folder: https://github.com/UZ-SLAMLab/ORB_SLAM3
Next, intsall LiTAMIN2 in the same directory as this folder: https://github.com/bzdfzfer/litamin2
Your folder structure should look something like this:
NA569_team_03_ORBSLAM_LiDAR/
	- ORB_SLAM3_DIR/
	- LiTAMIN2_dir/
	- README.md
	- Rest of files in repository

NOTE: You must ensure that the same version of C++ is used to compile both LiTAMIN2 and ORB-SLAM3. You may need to edit their respecitve CMakeLists.txt to force this

How to compile:

Move System.cc to ${ORB_SLAM_DIR}/src and overwrite
Move System.h to ${ORB_SLAM_DIR}/include

NOTE: After doing this, ORB-SLAM3 will no longer compile using its own CMakeLists.txt as the tests in their folder rely on the previous versions of the files

In CMakeLists.txt:
	Change lines 10 and 11 to wherever you have ORB_SLAM3 and litamin installed
	Change lines 160 and 161 to wherever you have libDBoW2.so and libg2o.so located. These only appear after you build ORBSLAM3 and should be located in the ${ORB_SLAM_DIR}/ThirdParty folder.

To build and compile, use the following commands inside the main directory:
	mkdir build
	cd build
	cmake ..
	make -j4

Files are output to the out/ folder in the main directory
Run the code using ./out/driver [single/double] path_to_vocabulary path_to_settings path_to_sequence path_to_kitti_dataset dataset_type slam_file litamin_file output_loc

The argument "single" corresponds to ORB-SLAM3 -> LiTAMIN2 and "double" corresponds to our synthesized method.
